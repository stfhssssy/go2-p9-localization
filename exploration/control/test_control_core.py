import unittest
from control_core import ControlPolicy

class PolicyTests(unittest.TestCase):
    def fresh(self, p, t=1., seq=1, **overrides):
        data=dict(node_instance_id='guard-a',status_sequence=seq,created_stamp_ns=int(t*1e9),mode='live',active=True,state='EXPLORING')
        data.update(overrides)
        p.robot(int(t*1e9),t,[.33,0,0,0])
        p.guard(data,t,int(t*1e9))
        p.velocity([.3,.2,0,0,0,.5],t)
    def armed(self):
        p=ControlPolicy(enabled=True);self.fresh(p);self.assertTrue(p.request_arm(1.));p.stop_ack(True)
        self.fresh(p,1.1,2);p.deadman(1.1);return p
    def test_preview_never_arms(self):
        p=ControlPolicy();self.fresh(p);self.assertFalse(p.request_arm(1));self.assertEqual(p.tick(1).kind,'idle')
    def test_requires_all_inputs(self):
        p=ControlPolicy(enabled=True);self.assertFalse(p.request_arm(1))
    def test_requires_ack(self):
        p=ControlPolicy(enabled=True);self.fresh(p);p.request_arm(1);p.deadman(1.1);self.fresh(p,1.1,2);self.assertEqual(p.tick(1.1).kind,'idle')
    def test_rejected_ack_latches(self):
        p=ControlPolicy(enabled=True);self.fresh(p);p.request_arm(1);p.stop_ack(False);self.assertEqual(p.tick(1).kind,'stop');self.assertEqual(p.state,'FAULT')
    def test_ack_does_not_move(self):
        p=ControlPolicy(enabled=True);self.fresh(p);p.request_arm(1);p.stop_ack(True);self.fresh(p,1.1,2);self.assertEqual(p.tick(1.1).kind,'idle')
    def test_new_inputs_required_after_arm(self):
        p=ControlPolicy(enabled=True);self.fresh(p);p.request_arm(1);p.stop_ack(True);p.deadman(1.1);self.assertEqual(p.tick(1.1).kind,'idle')
    def test_limits(self):
        p=self.armed();self.assertEqual(p.tick(1.1).velocity,(.15,.1,.25))
    def test_stale_command_stops(self):
        p=self.armed();self.assertEqual(p.tick(1.46).kind,'stop');self.fresh(p,1.5,3);p.deadman(1.5);self.assertEqual(p.tick(1.5).kind,'idle')
    def test_stale_state_stops(self):
        p=self.armed();p.velocity([0]*6,1.7);self.assertEqual(p.tick(1.7).kind,'stop')
    def test_repeated_robot_stamp_does_not_refresh(self):
        p=self.armed();p.robot(int(1.1e9),1.7,[0]*4);self.assertEqual(p.receipts['state'],1.1)
    def test_regressed_robot_stamp_stops(self):
        p=self.armed();p.robot(1,1.2,[0]*4);self.assertEqual(p.tick(1.2).kind,'stop')
    def test_deadman_stops_and_requires_rearm(self):
        p=self.armed();p.tick(1.1);self.fresh(p,1.8,3);self.assertEqual(p.tick(1.8).kind,'stop');self.fresh(p,1.9,4);p.deadman(1.9);self.assertEqual(p.tick(1.9).kind,'idle')
    def test_operator_stop(self):
        p=self.armed();p.stop();self.assertEqual(p.tick(1.1).kind,'stop')
    def test_guard_inactive(self):
        p=self.armed();self.fresh(p,1.2,3,active=False);self.assertEqual(p.tick(1.2).kind,'stop')
    def test_finish_disarms(self):
        p=self.armed();self.fresh(p,1.2,3,state='EXPLORATION_FINISHED');self.assertEqual(p.tick(1.2).kind,'stop')
    def test_repeated_status_disarms(self):
        p=self.armed();self.fresh(p,1.2,2);self.assertEqual(p.tick(1.2).kind,'stop')
    def test_guard_restart_disarms(self):
        p=self.armed();self.fresh(p,1.2,1,node_instance_id='guard-b');self.assertEqual(p.tick(1.2).kind,'stop')
    def test_old_stamp_rejected(self):
        p=self.armed();self.fresh(p,4,3,created_stamp_ns=1);self.assertEqual(p.tick(4).kind,'stop')
    def test_future_stamp_rejected(self):
        p=self.armed();self.fresh(p,1.2,3,created_stamp_ns=9000000000);self.assertEqual(p.tick(1.2).kind,'stop')
    def test_invalid_status_disarms(self):
        p=self.armed();p.guard({},1.2,1200000000);self.assertEqual(p.tick(1.2).kind,'stop')
    def test_sim_guard_disallowed(self):
        p=ControlPolicy(enabled=True);self.fresh(p,mode='simulation');self.assertFalse(p.request_arm(1))
    def test_nonfinite_and_nonplanar_rejected(self):
        for values in ([float('nan'),0,0,0,0,0],[0,0,1,0,0,0],[0]*5):
            p=self.armed();p.velocity(values,1.2);self.assertEqual(p.tick(1.2).kind,'stop')
    def test_bad_limits_rejected(self):
        for v in (float('nan'),float('inf'),0,-1,.6):
            with self.assertRaises(ValueError): ControlPolicy(max_forward=v)
    def test_competing_control_stops(self):
        p=self.armed();p.competing_control();self.assertEqual(p.tick(1.1).kind,'stop')
    def test_zeros_remain_zeros(self):
        p=self.armed();p.velocity([0]*6,1.2);self.assertEqual(p.tick(1.2).velocity,(0,0,0))
    def test_clock_regression_stops(self):
        p=self.armed();p.tick(1.2);self.assertEqual(p.tick(1.1).kind,'stop')
    def test_nonfinite_state_stops(self):
        p=self.armed();p.robot(2,1.2,[float('nan')]);self.assertEqual(p.tick(1.2).kind,'stop')

    def test_status_sequence_reset_same_instance_stops(self):
        p=self.armed();self.fresh(p,1.2,0);self.assertEqual(p.tick(1.2).kind,'stop')
    def test_new_guard_and_state_do_not_reuse_old_command(self):
        p=ControlPolicy(enabled=True);self.fresh(p);p.request_arm(1);p.stop_ack(True)
        p.robot(1100000000,1.1,[0]*4)
        p.guard(dict(node_instance_id='guard-a',status_sequence=2,created_stamp_ns=1100000000,mode='live',active=True,state='EXPLORING'),1.1,1100000000)
        p.deadman(1.1);self.assertEqual(p.tick(1.1).kind,'idle')
    def test_actual_guard_finish_contract(self):
        import sys
        from pathlib import Path
        root = Path(__file__).resolve().parent.parent
        runtime = root if (root/'guard_core.py').is_file() else root/'runtime'
        sys.path.insert(0,str(runtime))
        from guard_core import ExplorationGuard
        g=ExplorationGuard();t=10.
        def feed(t):
            ns=int(t*1e9);g.clock(ns,t)
            g.odometry(ns,'camera_init','base_link',(0,0,0),(0,0,0,1),t)
            for name in ('scan','terrain','terrain_ext'):g.cloud(name,ns,'camera_init',10,t)
            g.tracking(True,t);return ns
        feed(t);self.assertTrue(g.start(t)[0]);t=10.1;ns=feed(t)
        self.assertTrue(g.tare_waypoint(ns,'camera_init',(2,0,0),t))
        self.assertTrue(g.tare_path(ns,'camera_init',[(0,0,0),(2,0,0)],t))
        self.assertIsNotNone(g.take_waypoint(t))
        self.assertTrue(g.local_path(ns,'vehicle',[(0,0,0),(1,0,0)],t))
        self.assertTrue(g.command(ns,'vehicle',(.1,0,0),t));g.output(t)
        data=g.status(t);data.update(mode='live',node_instance_id='guard-a',status_sequence=3,created_stamp_ns=1200000000)
        p=self.armed();p.guard(data,1.2,1200000000);self.assertEqual(p.tick(1.2).kind,'move')
        g.finished(True);self.assertFalse(g.active);self.assertEqual(g.state,'EXPLORATION_FINISHED')
        data=g.status(t);data.update(mode='live',node_instance_id='guard-a',status_sequence=4,created_stamp_ns=1300000000)
        p.guard(data,1.3,1300000000);self.assertEqual(p.tick(1.3).kind,'stop')
        self.assertFalse(g.start(t)[0]);self.assertFalse(p.request_arm(1.3))

if __name__ == '__main__': unittest.main()
