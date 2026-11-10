"""Failures and causal sequencing are evaluated without a running ROS graph."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/go2_p9_navigation'))
from go2_p9_navigation.guard_core import NavigationGuard,GuardConfig


class GuardTests(unittest.TestCase):
    def ready(self):
        g=NavigationGuard();self.update(g,10.,1.);return g
    def update(self,g,t,w):
        s=round(t*1e9);g.clock(s,w)
        self.assertTrue(g.odometry(s,'camera_init','base_link',(0,0,.25),(0,0,0,1),w))
        for n in ('scan','terrain','terrain_ext'):self.assertTrue(g.cloud(n,s,'camera_init',100,w))
        g.tracking(True,w);g.graph(.02,w);g.graph_ready(True,w)
    def moving(self):
        g=self.ready();self.assertTrue(g.request_goal('camera_init',(2,0),1.)[0]);self.update(g,10.3,1.3)
        self.assertTrue(g.waypoint(10300000000,'camera_init',(1,0,.25),1.3))
        self.assertTrue(g.path(10300000000,'vehicle',[(0,0,0),(1,0,0)],1.3))
        self.assertTrue(g.command(10300000000,'vehicle',(.2,0,.1),1.3));return g
    def test_ready_requires_every_stream_and_tracking(self):
        g=self.ready();self.assertTrue(g.planning_ready(1.))
        for k in ('odom','scan','terrain','terrain_ext','graph','graph_ready','tracking'):
            g=self.ready();g.times.pop(k);self.assertFalse(g.planning_ready(1.));self.assertFalse(g.request_goal('camera_init',(1,0),1.)[0])
    def test_runtime_without_initialized_graph_rejects_goal(self):
        g=self.ready();g.graph_ready(False,1.);g.graph(.001,1.1)
        self.assertFalse(g.planning_ready(1.1))
        self.assertFalse(g.request_goal('camera_init',(1,0),1.1)[0])
        g.graph_ready(True,1.2);self.assertTrue(g.request_goal('camera_init',(1,0),1.2)[0])
        g.graph_ready(False,1.3);self.assertIsNone(g.goal);self.assertEqual(g.output(1.3),(0,0,0))

    def test_post_goal_waypoint_path_command_ordering(self):
        g=self.ready();g.request_goal('camera_init',(1,0),1.)
        self.assertFalse(g.path(10000000000,'vehicle',[(0,0,0)],1.));self.assertFalse(g.command(10000000000,'vehicle',(.2,0,0),1.))
        self.assertFalse(g.waypoint(10000000000,'camera_init',(1,0,0),1.));self.assertEqual(g.output(1.),(0,0,0))
        g=self.moving();self.assertEqual(g.output(1.3),(.2,0,.1))
    def test_tracking_loss_discards_goal(self):
        g=self.moving();g.tracking(False,1.31);self.assertEqual(g.output(1.31),(0,0,0));self.assertIsNone(g.goal)
        self.update(g,10.4,1.4);self.assertEqual(g.output(1.4),(0,0,0))
    def test_clock_pause_and_backwards_are_stops(self):
        g=self.moving();g.clock(10300000000,1.9);self.assertEqual(g.output(1.9),(0,0,0));self.assertIsNone(g.goal)
        g=self.moving();g.clock(10000000000,1.4);self.assertTrue(g.clock_reset_latched)
        self.update(g,10.4,1.5);self.assertFalse(g.request_goal('camera_init',(1,0),1.5)[0])
    def test_stale_command_is_zero_with_fresh_sensors(self):
        g=self.moving();self.update(g,10.9,1.9);self.assertEqual(g.output(1.9),(0,0,0))
    def test_stale_cloud_and_invalid_odom_stop(self):
        g=self.moving();g.cloud('scan',0,'camera_init',100,1.31);self.assertEqual(g.output(1.31),(0,0,0))
        g=self.moving();g.odometry(10310000000,'map','base_link',(0,0,0),(0,0,0,1),1.31);self.assertIsNone(g.goal)
    def test_empty_nonfinite_wrong_frame_paths_stop(self):
        for frame,path in [('vehicle',[]),('camera_init',[(0,0,0)]),('vehicle',[(float('nan'),0,0)])]:
            g=self.moving();self.assertFalse(g.path(10300000001,frame,path,1.31));self.assertEqual(g.output(1.31),(0,0,0))
    def test_goal_frame_distance_finiteness_and_stop(self):
        g=self.ready()
        for frame,xy in [('map',(1,0)),('camera_init',(float('nan'),0)),('camera_init',(31,0))]:self.assertFalse(g.request_goal(frame,xy,1.)[0])
        g=self.moving();g.stop();self.assertEqual(g.output(1.31),(0,0,0));self.assertFalse(g.command(10300000000,'vehicle',(.2,0,0),1.31))
    def test_limits_reached_guard_and_retarget(self):
        g=self.moving();g.command(10300000000,'vehicle',(3,-3,5),1.3);self.assertEqual(g.output(1.3),(.25,-.2,.5))
        self.assertFalse(g.reached(True));self.assertTrue(g.request_goal('camera_init',(3,0),1.3)[0]);self.assertEqual(g.output(1.3),(0,0,0))
    def test_duplicate_cached_frame_does_not_refresh_watchdog(self):
        g=self.ready();self.assertTrue(g.cloud('terrain',10000000000,'camera_init',100,1.4))
        self.assertEqual(g.times['terrain'],1.)
        g.clock(11000000000,2.);self.assertFalse(g.cloud('terrain',10000000000,'camera_init',100,2.))
        self.assertNotIn('terrain',g.times)

    def test_invalid_configuration(self):
        for kw in [dict(max_forward_m_s=5),dict(clock_timeout_sec=0),dict(goal_gate_sec=float('nan'))]:
            with self.assertRaises(ValueError):GuardConfig(**kw)

if __name__=='__main__':unittest.main()
