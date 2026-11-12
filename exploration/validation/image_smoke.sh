#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/unitree_ros_ws/install/setup.bash ]]; then source /opt/unitree_ros_ws/install/setup.bash; fi
source /opt/p9_ws/install/setup.bash
source /opt/p9_control_ws/install/local_setup.bash
source /tare_ws/install/local_setup.bash
export LD_LIBRARY_PATH="/tare_ws/src/tare_planner/or-tools/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export P9_EXPLORATION_HOME=/opt/p9-exploration
python3 -m unittest discover -s /opt/p9-exploration -p 'test_guard_core.py'
python3 -m unittest discover -s /opt/p9-exploration -p 'test_exploration_config.py'
python3 -m unittest discover -s /opt/p9-exploration/control -p 'test_control_*.py'
python3 /opt/p9-exploration/test_observe_exploration.py
python3 - <<'PY'
from unitree_api.msg import Request,Response
from unitree_go.msg import SportModeState
from go2_p9.configuration import load
from pathlib import Path
import hashlib
profile=Path('/opt/p9/config/p9nav-live.yaml')
assert hashlib.sha256(profile.read_bytes()).hexdigest() == '663b31f4cd230e06b842e2d83ab5f4d00211420a70e91a28e7ac7736c35c2984'
core,sensor,mapping,pointlio=load(profile,'live')
assert not sensor['use_sim_time'] and not mapping['use_sim_time'] and not pointlio['use_sim_time']
assert pointlio['mapping']['extrinsic_R']==core['calibration']['rotation']
assert pointlio['mapping']['extrinsic_T']==core['calibration']['translation']
assert sensor['gyro_bias']==[0.00028568964388210946,-0.0018381138448739292,0.0005906477368247819]
assert sensor['imu_mode']=='raw_linear'
print('Live C R/t + B1 gyro configuration and Unitree message imports verified')
PY
p9explore live --boundary -2 2 -2 2 --output /tmp/p9-explore-build-check --validate-only
p9explore --help
p9explore-control --help
/opt/p9-tare-provenance/tare-tsp-smoke

python3 /opt/p9-exploration/validation/test_tare_frontier_source_contract.py /tare_ws
python3 /opt/p9-exploration/validation/test_tare_frontier_parameter.py --timeout 60 --output /opt/p9-tare-provenance/frontier-parameter-test.json
