#!/usr/bin/env python3
"""Counts cube placements in a workspace-camera recording, without a person watching.

The workspace camera looks down into the arena: a wooden floor with four inclined coloured surfaces
around it. A cube put on a surface slides back onto the floor. On the surface of its own colour the
cube is hard to see (red on red), so the detector does not look for it there. It follows the cube
where it stands out, on the floor, and counts a placement when the cube

    1. reaches the surface of its own colour: it is seen right at the edge of the floor next to
       that surface, or it leaves the floor there while moving towards the surface and comes back
       there, and
    2. within 3 seconds is back on the floor away from the surface, either shooting away from it
       while mostly uncovered (in the hand it is mostly hidden) or staying in one place, and not
       on the spot where it lay before (then the hand only covered it).

Only the surface of the cube's own colour is looked at: a cube put on another surface is simply
not counted.

    python eval_detect.py [--json] <oakd_front_view.mp4 | processed demo .h5> [...]
    python eval_detect.py --live     (used by APE: frames in on stdin, one line of JSON out per frame)

The same rule runs in two ways. After a run, on the whole video: the arena is taken from frames
spread over all of it. Live, frame by frame: the arena is taken from the frames seen so far and
refreshed as the run goes on, and the cube's usual size is the one seen so far.

Built and checked on the red-cube demos only (see README): blue and yellow cubes, daylight, and how
often a failed attempt is counted by mistake are not known yet.
"""
import json
import struct
import sys

import cv2
import numpy as np

W = 640                     # frames are resized to this width; the pixel values below belong to it
FPS = 10.0
# OpenCV HSV (H 0..179): hue ranges, minimum saturation, minimum value
COLOURS = {
    "red": {"hue": [(0, 8), (172, 180)], "s": 150, "v": 70},
    "blue": {"hue": [(95, 128)], "s": 120, "v": 50},
    "yellow": {"hue": [(21, 32)], "s": 215, "v": 185},   # the wooden floor has hue 18, but is darker and less saturated
}
WOOD = {"hue": [(13, 22)], "s": 110, "v": 60, "v_max": 215}
MIN_AREA, MAX_AREA = 150, 9000   # the cube covers about 3000 pixels when nothing hides it
BORDER = 15                 # px: floor pixels this close to its edge are ignored where the background has the cube's colour (surface bleeding in)
TOUCH = 15                  # px: cube centre this close to a surface = it is on the edge of the floor, at the surface
LEAVE = 32                  # px: last seen this close to a surface, and approaching it, before the cube was gone = it went onto the surface
NEAR, NEAR_LONG = 60, 130   # px: how close to the surface it comes back; the larger value after a longer absence (it slid on while hidden)
LONG_GAP = 3                # frames: a "longer absence"
FREE = 18                   # px: a cube lying on the floor is at least this far from the surface
MOVED = 8                   # px between where a cube seen at the edge touched and where it ended up
AREA_FREE = 0.35            # of its usual size: a cube lying free shows at least this much; in the hand it is mostly hidden
SAME_SPOT = 10              # px: ending up this close to where it lay before is not a placement (the hand only covered it)
SLIDE_PX, SLIDE_FRAMES = 30, 4   # sliding: this far away from the surface within this many frames (the hand carries at about 13 px per frame)
REST_PX, REST_SEEN, REST_FRAMES = 12.0, 5, 8   # staying in one place: seen REST_SEEN times in REST_FRAMES frames, all within REST_PX
CONFIRM_S = 3.0             # how soon after being at the surface the cube must be back on the floor
MIN_BETWEEN_S = 2.5         # two placements cannot follow each other faster (the fastest in the teleoperated demos: 3.1 s)
MIN_SPREAD = 40             # px: the cube is the coloured thing that moves at least this far in the recording

def _mask(hsv, spec):
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    hue = np.zeros(h.shape, bool)
    for lo, hi in spec["hue"]:
        hue |= (h >= lo) & (h <= hi)
    m = hue & (s >= spec["s"]) & (v >= spec["v"])
    if "v_max" in spec:
        m &= v <= spec["v_max"]
    return m.astype(np.uint8)


def _small(rgb):
    h, w = rgb.shape[:2]
    return rgb if w == W else cv2.resize(rgb, (W, int(round(h * W / w))), interpolation=cv2.INTER_AREA)


def iter_frames(source):
    """The frames of a recording as RGB, resized to width W, one at a time (a long run does not fit in memory)."""
    if isinstance(source, list):
        yield from source
    elif str(source).endswith((".h5", ".hdf5")):
        import h5py
        with h5py.File(source, "r") as f:
            ds = f["observations/images/oakd_front_view/color"]
            for i in range(len(ds)):
                yield _small(ds[i])
    else:
        cap = cv2.VideoCapture(str(source))
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            yield _small(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        cap.release()


def read_frames(source):
    return list(iter_frames(source))


def _background(sample):
    """What stays the same over the frames given: (the floor as a filled outline, its outline, per colour where the image has that colour)."""
    background = np.median(np.stack(sample), axis=0).astype(np.uint8)
    hsv = cv2.cvtColor(background, cv2.COLOR_RGB2HSV)
    wood = cv2.morphologyEx(_mask(hsv, WOOD), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(wood, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    hull = cv2.convexHull(max(contours, key=cv2.contourArea))
    if cv2.contourArea(hull) < 0.05 * wood.size:
        return None
    floor = np.zeros(wood.shape, np.uint8)
    cv2.fillConvexPoly(floor, hull, 1)
    return floor, hull, {name: _mask(hsv, spec) for name, spec in COLOURS.items()}


def _build(floor, hull, colours):
    """The arena as the detector uses it: the floor, where a surface's colour reaches into its edge, and the distance to each surface."""
    around = cv2.dilate(floor, np.ones((31, 31), np.uint8)) & (1 - floor)   # a band just outside the floor
    border = floor & (1 - cv2.erode(floor, np.ones((2 * BORDER + 1, 2 * BORDER + 1), np.uint8)))
    surfaces, static = {}, {}
    for name, full in colours.items():
        static[name] = cv2.dilate(full & border, np.ones((5, 5), np.uint8))   # the surface's colour reaching into the floor's edge
        m = cv2.morphologyEx(full & (1 - floor), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(m)
        best = max((i for i in range(1, n) if (around[labels == i]).any()), key=lambda i: stats[i, cv2.CC_STAT_AREA], default=None)
        if best is not None and stats[best, cv2.CC_STAT_AREA] > 0.01 * m.size:
            surfaces[name] = cv2.distanceTransform((labels != best).astype(np.uint8), cv2.DIST_L2, 3)   # distance of every pixel to this surface
    return {"floor": cv2.erode(floor, np.ones((7, 7), np.uint8)), "surfaces": surfaces, "static": static, "floor_share": round(float(floor.mean()), 3),
            "hull": hull.reshape(-1, 2).tolist(), "size": [int(floor.shape[1]), int(floor.shape[0])]}


def arena(sample):
    """The floor and the surfaces around it, from what stays the same over the recording (`sample`: frames spread over it)."""
    found = _background(sample)
    return None if found is None else _build(*found)


def locate(rgb, a, colours):
    """Per colour: (x, y, area) of a cube of that colour on the floor in this frame, or None."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    out = {}
    for c in colours:
        n, _, stats, centres = cv2.connectedComponentsWithStats(_mask(hsv, COLOURS[c]) & a["floor"] & (1 - a["static"][c]))
        best = max(range(1, n), key=lambda i: stats[i, cv2.CC_STAT_AREA], default=None)
        ok = best is not None and MIN_AREA <= stats[best, cv2.CC_STAT_AREA] <= MAX_AREA
        if ok:   # a long thin strip is a surface's edge, not the cube
            w, h = stats[best, cv2.CC_STAT_WIDTH], stats[best, cv2.CC_STAT_HEIGHT]
            ok = min(w, h) >= 8 and max(w, h) <= 3.5 * min(w, h)
        out[c] = (float(centres[best][0]), float(centres[best][1]), int(stats[best, cv2.CC_STAT_AREA])) if ok else None
    return out


def track(frames, a, colours):
    """Per colour and frame: where the cube is (see locate)."""
    out = {c: [] for c in colours}
    for rgb in frames:
        for c, p in locate(rgb, a, colours).items():
            out[c].append(p)
    return out


def _distance(a, colour):
    """p -> distance in px of a cube position to the surface of that colour, or None if that surface was not found."""
    surface = a["surfaces"].get(colour) if a else None
    return None if surface is None else (lambda p: float(surface[int(round(p[1])), int(round(p[0]))]))


class Counter:
    """The counting rule for one cube colour, one frame at a time. `placed` holds the frame numbers of the placements."""

    def __init__(self, fps=FPS, trace=None):
        self.fps, self.tr, self.placed = fps, [], []
        self.touch = None            # (frame, x, y, seen at the edge) of the last time the cube was at the surface
        self.last = None             # the frame in which the cube was last seen
        self.lay = self.lay_before = None   # where the cube last stayed in one place; the same, as it was when it reached the surface
        self.last_event = -1e9
        self.areas, self.seen, self.box = [], 0, None
        self.note = (lambda *a: trace.append(a)) if trace is not None else (lambda *a: None)

    @property
    def spread(self):
        return float(np.hypot(self.box[2] - self.box[0], self.box[3] - self.box[1])) if self.box else 0.0

    def _step(self, i, j):
        return np.hypot(self.tr[i][0] - self.tr[j][0], self.tr[i][1] - self.tr[j][1])

    def _stays(self, i, first):
        window = [j for j in range(max(first, i - REST_FRAMES + 1), i + 1) if self.tr[j] is not None]
        return sum(self._step(i, j) <= REST_PX for j in window) >= REST_SEEN

    def step(self, p, dist, typical=None):
        """p: the cube in this frame or None. dist: see _distance. typical: the cube's size when nothing hides it
        (None = the size seen so far). Returns True if this frame completed a placement."""
        tr, fps = self.tr, self.fps
        tr.append(p)
        i = len(tr) - 1
        if p is None:
            return False
        self.seen += 1
        self.box = (p[0], p[1], p[0], p[1]) if self.box is None else (min(self.box[0], p[0]), min(self.box[1], p[1]), max(self.box[2], p[0]), max(self.box[3], p[1]))
        if typical is None:
            self.areas.append(p[2])
            typical = float(np.percentile(self.areas[-3000:], 80))
        if dist is None:
            self.last = i
            return False
        counted = False
        d, fresh, last = dist(p), self.touch is None, self.last
        if d <= TOUCH:
            self.touch = (i, p[0], p[1], True)
            self.note(i, "at edge")
        elif last is not None and i - last > 1 and dist(tr[last]) <= LEAVE and d <= (NEAR_LONG if i - last > LONG_GAP else NEAR):
            before = [j for j in range(max(0, last - 4), last) if tr[j] is not None]   # gone at the surface and back there:
            if before and dist(tr[before[0]]) - dist(tr[last]) >= 8:                   # onto it, if the cube was approaching it when it went
                self.touch = (i, p[0], p[1], False)
                self.note(i, "gone and back", "last seen", last)
        touch = self.touch
        if touch is not None and fresh:
            self.lay_before = self.lay
        if touch is not None and (i - touch[0]) / fps > CONFIRM_S:
            touch = self.touch = None
        if touch is None and d > TOUCH and self._stays(i, 0):
            self.lay = (p[0], p[1])
        if touch is not None and i > touch[0] and d >= FREE and touch[0] / fps - self.last_event >= MIN_BETWEEN_S:
            since = [j for j in range(touch[0], i) if tr[j] is not None]
            slid = p[2] >= AREA_FREE * typical and any(i - j <= SLIDE_FRAMES and d - dist(tr[j]) >= SLIDE_PX for j in since)
            stayed = self._stays(i, touch[0] + 1)
            same = self.lay_before is not None and np.hypot(p[0] - self.lay_before[0], p[1] - self.lay_before[1]) <= SAME_SPOT
            if slid or stayed:
                self.note(i, "back on the floor", "slid" if slid else "stayed", "same spot" if same else "")
            if (slid or stayed) and not same and (not touch[3] or np.hypot(p[0] - touch[1], p[1] - touch[2]) >= MOVED):
                self.placed.append(touch[0])
                self.last_event, self.touch, counted = touch[0] / fps, None, True
            elif stayed and same:
                self.touch = None
        self.last = i
        return counted


def _cube_colour(counters):
    """The cube is the coloured thing on the floor that moves, and of those the one seen most."""
    moving = [c for c in counters if counters[c].spread >= MIN_SPREAD] or list(counters)
    return max(moving, key=lambda c: counters[c].seen)


def detect(path_or_frames, colour=None, fps=FPS, trace=None):
    """A whole recording -> {"colour", "placed": [seconds], "problem", "frames", "seen_share"}"""
    n = sum(1 for _ in iter_frames(path_or_frames)) if not isinstance(path_or_frames, list) else len(path_or_frames)
    out = {"colour": colour, "placed": [], "problem": "", "frames": n}
    if n < 20:
        out["problem"] = "the recording is too short"
        return out
    every = max(1, n // 40)
    a = arena([f for k, f in enumerate(iter_frames(path_or_frames)) if k % every == 0])
    if a is None:
        out["problem"] = "the arena floor was not found in the image"
        return out
    tracks = track(iter_frames(path_or_frames), a, [colour] if colour else list(COLOURS))
    if colour is None:
        seen = {c: Counter() for c in tracks}
        for c, tr in tracks.items():
            for p in tr:
                seen[c].step(p, None, 1.0)
        colour = out["colour"] = _cube_colour(seen)
    tr = tracks[colour]
    out["seen_share"] = round(sum(p is not None for p in tr) / len(tr), 2)
    if out["seen_share"] < 0.1:
        out["problem"] = "no cube was seen on the floor"
        return out
    dist = _distance(a, colour)
    if dist is None:
        out["problem"] = f"the {colour} surface was not found next to the floor"
        return out
    typical = float(np.percentile([p[2] for p in tr if p is not None], 80))   # the cube's size in the image when nothing hides it
    counter = Counter(fps, trace)
    for p in tr:
        counter.step(p, dist, typical)
    out["placed"] = [round(i / fps, 1) for i in counter.placed]
    return out


class Live:
    """The same count while the frames come in. The arena is taken from the last 30 seconds (one frame per
    second) and looked for again after 1, 3, 6 and 12 s and then every 10 s. Only recent frames are used
    because the arena does not always stay where it is: in one demo the robot pushed it 3 cm aside."""

    def __init__(self, fps=FPS):
        self.fps, self.n, self.sample = fps, 0, []
        self.arena, self.next_arena = None, 10
        self.reset()

    def reset(self):
        """A new run begins: the count starts at zero, what is known about the arena stays."""
        self.counters = {c: Counter(self.fps) for c in COLOURS}
        self.stamps = []

    def add(self, rgb, stamp):
        self.n += 1
        if self.n <= 10 or self.n % 10 == 0:   # every frame of the first second, then one per second
            self.sample = (self.sample + [rgb])[-30:]
        out = {"n": self.n, "colour": None, "cube": None, "placed": [], "at_surface": False, "problem": ""}
        if self.n >= self.next_arena:
            found = arena(self.sample)
            if found is not None:
                self.arena = found
                out["hull"], out["size"] = found["hull"], found["size"]
            self.next_arena = self.n + (20 if self.n < 30 else 30 if self.n < 60 else 60 if self.n < 120 else 100)
        self.stamps.append(stamp)
        if self.arena is None:
            for counter in self.counters.values():
                counter.step(None, None)
            out["problem"] = "the arena floor is not found in the image yet" if self.n >= 10 else ""
            return out
        where = locate(rgb, self.arena, list(COLOURS))
        for c, counter in self.counters.items():
            counter.step(where[c], _distance(self.arena, c))
        colour = _cube_colour(self.counters)
        counter = self.counters[colour]
        if not counter.seen:
            out["problem"] = "no cube is seen on the floor"
            return out
        out.update(colour=colour, cube=where[colour], placed=[self.stamps[i] for i in counter.placed], at_surface=counter.touch is not None)
        if _distance(self.arena, colour) is None:
            out["problem"] = f"the {colour} surface is not found next to the floor"
        return out


def detect_live(source, fps=FPS):
    """A whole recording, counted the way it would have been counted live (to compare the two ways)."""
    live, out = Live(fps), None
    for i, rgb in enumerate(iter_frames(source)):
        out = live.add(rgb, i / fps)
    return {"colour": out["colour"], "placed": [round(t, 1) for t in out["placed"]], "problem": out["problem"], "frames": live.n} if out else {"placed": [], "problem": "no frames"}


def live_main():
    """APE's helper process. In, repeated: 16 bytes (time as float64, width and height as uint16, a uint32 that is
    1 for "a new run begins" and else 0), then width x height x 3 bytes RGB unless it was 1. Out: one line of JSON per frame."""
    live, inp = Live(), sys.stdin.buffer
    while True:
        head = inp.read(16)
        if len(head) < 16:
            return
        stamp, w, h, flag = struct.unpack("<dHHI", head)
        if flag == 1:
            live.reset()
            continue
        data = inp.read(w * h * 3)
        if len(data) < w * h * 3:
            return
        try:
            state = live.add(_small(np.frombuffer(data, np.uint8).reshape(h, w, 3)), stamp)
        except Exception as exc:   # one bad frame must not end the count
            state = {"problem": f"{type(exc).__name__}: {exc}"}
        state["t"] = stamp
        sys.stdout.write(json.dumps(state) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    if "--live" in sys.argv:
        live_main()
        sys.exit(0)
    as_json = "--json" in sys.argv
    for path in [x for x in sys.argv[1:] if x != "--json"]:
        r = detect(path)
        if as_json:
            print(json.dumps(r))
        else:
            print(f"{path}: {len(r['placed'])} placed at {r['placed']} | colour {r['colour']} | seen {r.get('seen_share')} | {r['problem']}")
