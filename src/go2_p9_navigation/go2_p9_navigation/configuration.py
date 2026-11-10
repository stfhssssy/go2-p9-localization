"""Fixed planner interfaces and validated, reproducible parameter overrides."""
from pathlib import Path
import copy
import math
import yaml
from .guard_core import GuardConfig

SECTIONS=('local_planner','path_follower','terrain_analysis','terrain_analysis_ext','far_planner')
OWNED={'use_sim_time','world_frame','output_frame','is_real_robot','pathFolder','sensorOffsetX','sensorOffsetY','publish_input_ack'}


def share():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('go2_p9_navigation'))


def load_navigation(path, mode, sim_command_output=False, defaults_path=None, core_config=None):
    if mode not in ('replay','live','sim'):raise ValueError('Invalid mode')
    if sim_command_output and mode!='sim':raise ValueError('--sim-command-output is only valid with sim')
    defaults=yaml.safe_load(Path(defaults_path or share()/'config/planner-defaults.yaml').read_text())
    cfg=yaml.safe_load(Path(path).read_text())
    if not isinstance(cfg,dict) or type(cfg.get('version')) is not int or cfg['version']!=1:raise ValueError('Expected navigation version: 1')
    if set(cfg)-{'version','guard',*SECTIONS}:raise ValueError('Unknown navigation section')
    out=copy.deepcopy(defaults)
    if mode=='sim':out['terrain_analysis']['noDataObstacle']=False
    for section in SECTIONS:
        overrides=cfg.get(section,{})
        if not isinstance(overrides,dict):raise ValueError(section+' must be a mapping')
        for key,value in overrides.items():
            if key in OWNED or key not in out[section]:raise ValueError('Unknown or reserved planner parameter: '+section+'.'+key)
            original=out[section][key]
            if isinstance(original,bool):valid=type(value) is bool
            elif isinstance(original,int):valid=type(value) is int
            elif isinstance(original,float):valid=type(value) in (int,float) and math.isfinite(value)
            else:valid=isinstance(value,str)
            if not valid:raise ValueError('Wrong type/nonfinite planner parameter: '+key)
            out[section][key]=float(value) if isinstance(original,float) else value
    for name in ('local_planner','path_follower'):
        for key in ('maxSpeed','autonomySpeed'):
            if not 0<out[name][key]<=1.:raise ValueError(name+'.'+key+' must be in (0, 1] m/s')
        if not out[name]['autonomyMode']:raise ValueError('This entry requires autonomyMode')
    if not 0<out['path_follower']['maxYawRate']<=114.6:raise ValueError('maxYawRate must be in (0, 114.6] degrees/s')
    for name in ('terrain_analysis','terrain_analysis_ext'):
        out[name].update(output_frame='camera_init',publish_input_ack=False)
    for params in out.values():params['use_sim_time']=mode!='live'
    guard=GuardConfig(**cfg.get('guard',{}))
    if core_config is not None:
        m=core_config.get('mapping',{})
        if not m.get('publish_base_frame') or not m.get('publish_terrain_inputs') or m.get('base_frame','base_link')!='base_link':
            raise ValueError('Navigation requires core mapping publish_base_frame, publish_terrain_inputs and base_frame=base_link')
    return out,guard
