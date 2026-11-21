"""Write a resolved training config as rwr_il saves it, without training."""
import sys
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf
from srl_il.common.omegaconf import register_common_resolvers, resolve_srl_il_proj_cfg_root
import os
os.environ.setdefault("RWR_IL_DATA_DIR", "/data/datasets/x"); os.environ.setdefault("RWR_IL_CACHE_DIR", "/data/cache")
register_common_resolvers()
out, name, dataset = sys.argv[1:4]
root = resolve_srl_il_proj_cfg_root()
print("cfg root", root)
with initialize_config_dir(config_dir=os.path.abspath(root) if not os.path.isabs(root) else root, version_base=None):
    cfg = compose(config_name=name, overrides=[f"dataset_name={dataset}"] + sys.argv[4:], return_hydra_config=True)
    from hydra.core.hydra_config import HydraConfig
    HydraConfig.instance().set_config(cfg)
    c = {k: (OmegaConf.to_container(cfg[k], resolve=True) if OmegaConf.is_config(cfg[k]) else cfg[k]) for k in cfg if k != "hydra"}
    OmegaConf.save(OmegaConf.create(c), out)
