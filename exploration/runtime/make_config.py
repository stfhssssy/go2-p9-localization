#!/usr/bin/env python3
"""Pure, validated configuration helpers; never execute a simulation at import."""
from dataclasses import asdict
from pathlib import Path
import copy
import math
import yaml
from guard_core import GuardConfig

TARE_OVERRIDES = {
 'sub_state_estimation_topic_':'/state_estimation',
 'sub_start_exploration_topic_':'/start_exploration',
 'sub_coverage_boundary_topic_':'/p9/exploration/coverage_boundary',
 'sub_viewpoint_boundary_topic_':'/p9/exploration/navigation_boundary',
 'pub_waypoint_topic_':'/p9/exploration/tare_waypoint',
 'pub_exploration_finish_topic_':'/p9/exploration/finished',
 'pub_runtime_topic_':'/p9/exploration/tare_runtime',
 'kAutoStart':False,'kRushHome':False,'kRushHomeDist':3.0,'kExtendWayPoint':False,
 'kLookAheadDistance':1.5,'kAtHomeDistThreshold':0.4,
 'kUseCoverageBoundaryOnFrontier':True,'kUseCoverageBoundaryOnObjectSurface':True,
 'kSurfaceCloudDwzLeafSize':0.2,'kCollisionCloudDwzLeafSize':0.1,
 'kKeyposeCloudDwzFilterLeafSize':0.1,'kKeyposeCloudStackNum':3,
 'kPointCloudRowNum':15,'kPointCloudColNum':15,'kPointCloudLevelNum':5,
 'kPointCloudCellSize':3.0,'kPointCloudCellHeight':1.0,
 'kGridWorldXNum':31,'kGridWorldYNum':31,'kGridWorldZNum':5,
 'kGridWorldCellHeight':2.0,'kGridWorldNearbyGridNum':5,
 'viewpoint_manager/number_x':30,'viewpoint_manager/number_y':30,
 'viewpoint_manager/resolution_x':0.4,'viewpoint_manager/resolution_y':0.4,
 'kViewPointCollisionMargin':0.35,'kViewPointCollisionMarginZPlus':0.45,
 'kViewPointCollisionMarginZMinus':0.2,'kViewPointHeightFromTerrain':0.32,
 'kViewPointHeightFromTerrainChangeThreshold':0.4,
 'kSensorRange':5.0,'kNeighborRange':1.5,'kTerrainCollisionThreshold':0.18,
 'kFrontierClusterTolerance':0.5,'kFrontierClusterMinSize':8,
 'kCoverageDilationRadius':0.3,'kCoverageOcclusionThr':0.2,
 'rolling_occupancy_grid/resolution_x':0.2,'rolling_occupancy_grid/resolution_y':0.2,
 'rolling_occupancy_grid/resolution_z':0.2,
 'kMinAddPointNumSmall':15,'kMinAddPointNumBig':25,'kMinAddFrontierPointNum':5,
 'keypose_graph/kAddNodeMinDist':0.4,'keypose_graph/kAddEdgeCollisionCheckResolution':0.2,
 'keypose_graph/kAddEdgeCollisionCheckRadius':0.25,
}


def validate_options(mode, boundary, forward=.20, lateral=.10, yaw=.35,
                     sim_command_output=False, sim_auto_resume_inputs=False,
                     allow_return_home=False):
    if mode not in ('live', 'replay', 'sim'):
        raise ValueError('Mode must be live, replay or sim')
    if boundary is None or len(boundary) != 4:
        raise ValueError('Explicit --boundary MIN_X MAX_X MIN_Y MAX_Y required (camera_init metres)')
    if sim_command_output and mode != 'sim':
        raise ValueError('--sim-command-output is only valid in sim')
    if sim_auto_resume_inputs and not (mode == 'sim' and sim_command_output):
        raise ValueError('--sim-auto-resume-inputs requires sim with --sim-command-output')
    if allow_return_home and mode != 'sim':
        raise ValueError('Automatic return remains an unverified simulation-only experiment')
    guard = GuardConfig(min_x=boundary[0], max_x=boundary[1], min_y=boundary[2], max_y=boundary[3],
                        max_forward_m_s=forward, max_lateral_m_s=lateral, max_yaw_rad_s=yaw)
    return asdict(guard)


def tare_parameters(source, mode, allow_return_home=False):
    if allow_return_home and mode != 'sim':
        raise ValueError('Return-home is restricted to simulation')
    document = yaml.safe_load(Path(source).read_text())
    values = copy.deepcopy(document['tare_planner_node']['ros__parameters'])
    values.update(TARE_OVERRIDES)
    values.update(use_sim_time=mode != 'live', kRushHome=allow_return_home)
    return {'tare_planner_node': {'ros__parameters': values}}


def core_profile(core, estimator_path):
    """Preserve all calibrated sensor/extrinsic values; only throttle map display."""
    result = copy.deepcopy(core)
    result.setdefault('mapping', {}).update(map_publish_period_sec=5.0, path_publish_period_sec=1.0)
    result['estimator'] = {'config': str(Path(estimator_path).resolve())}
    return result


def local_parameters(parameters, guard):
    result = {key: copy.deepcopy(value) for key, value in parameters.items() if key != 'far_planner'}
    result['path_follower']['stopDisThre'] = .10
    for key in ('local_planner', 'path_follower'):
        result[key]['maxSpeed'] = min(result[key]['maxSpeed'], guard['max_forward_m_s'])
        result[key]['autonomySpeed'] = min(result[key]['autonomySpeed'], guard['max_forward_m_s'])
    result['path_follower']['maxYawRate'] = min(result['path_follower']['maxYawRate'],
                                              math.degrees(guard['max_yaw_rad_s']))
    return result
