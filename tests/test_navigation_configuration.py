from pathlib import Path
import sys
import tempfile
import unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/go2_p9_navigation'))
from go2_p9_navigation.configuration import load_navigation
BASE=ROOT/'src/go2_p9_navigation/config'

class NavigationConfigurationTests(unittest.TestCase):
    def load(self,mode='replay',bridge=False,overrides=None,core=None):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'nav.yaml';path.write_text(yaml.safe_dump(overrides or {'version':1}))
            return load_navigation(path,mode,bridge,BASE/'planner-defaults.yaml',core)
    def test_sim_only_bridge_and_sim_no_data_policy(self):
        for mode in ['live','replay']:
            with self.assertRaises(ValueError):self.load(mode,True)
        sim,_=self.load('sim',True);replay,_=self.load()
        self.assertFalse(sim['terrain_analysis']['noDataObstacle']);self.assertTrue(replay['terrain_analysis']['noDataObstacle'])
    def test_frames_and_unitree_are_owned(self):
        for section,key,val in [('far_planner','world_frame','map'),('path_follower','is_real_robot',True),('local_planner','pathFolder','/tmp')]:
            with self.assertRaises(ValueError):self.load(overrides={'version':1,section:{key:val}})
    def test_profile_must_supply_body_and_terrain_outputs(self):
        with self.assertRaises(ValueError):self.load(core={'mapping':{}})
        params,_=self.load(core={'mapping':{'publish_base_frame':True,'publish_terrain_inputs':True}})
        self.assertEqual(params['terrain_analysis']['output_frame'],'camera_init')
    def test_ros_types_and_finite_overrides(self):
        with self.assertRaises(ValueError):self.load(overrides={'version':1,'local_planner':{'maxSpeed':float('nan')}})
        with self.assertRaises(ValueError):self.load(overrides={'version':1,'local_planner':{'autonomyMode':'false'}})
        p,_=self.load(overrides={'version':1,'local_planner':{'maxSpeed':1}});self.assertIsInstance(p['local_planner']['maxSpeed'],float)

if __name__=='__main__':unittest.main()

class NavigationSourceContractTests(unittest.TestCase):
    def test_pointlio_path_isolated_from_controller(self):
        import ast
        tree=ast.parse((ROOT/'src/go2_p9/launch/go2_p9.launch.py').read_text())
        pairs=[tuple(v.value for v in n.elts) for n in ast.walk(tree) if isinstance(n,ast.Tuple) and len(n.elts)==2 and all(isinstance(v,ast.Constant) for v in n.elts)]
        self.assertIn(('/path','/pointlio/path'),pairs)
    def test_follower_has_no_hardware_publisher(self):
        text=(ROOT/'src/local_planner/src/pathFollower.cpp').read_text()
        self.assertNotIn('/api/sport/request',text)
        self.assertNotIn('SportClient',text)
        self.assertIn('create_publisher<geometry_msgs::msg::TwistStamped>',text)
