#!/usr/bin/env python3
"""Offline behavioral regression tests for the TARE command gate."""
import unittest
from itertools import permutations
from guard_core import ExplorationGuard, GuardConfig


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.g = ExplorationGuard()
        self.t = 10.
        self.feed()

    def feed(self, dt=.1, pos=(0., 0., 0.), clouds=True):
        self.t += dt
        ns = round(self.t*1e9)
        self.g.clock(ns, self.t)
        self.g.odometry(ns, 'camera_init', 'base_link', pos, (0., 0., 0., 1.), self.t)
        if clouds:
            for name in ('scan', 'terrain', 'terrain_ext'):
                self.g.cloud(name, ns, 'camera_init', 10, self.t)
        self.g.tracking(True, self.t)
        return ns

    def start(self):
        self.assertTrue(self.g.start(self.t)[0])

    def plan(self, target=(3., 0., 0.)):
        ns = self.feed()
        self.assertTrue(self.g.tare_waypoint(ns, 'camera_init', target, self.t))
        self.assertTrue(self.g.tare_path(ns, 'camera_init', [(0., 0., 0.), target], self.t))
        self.assertIsNotNone(self.g.take_waypoint(self.t))
        self.assertTrue(self.g.local_path(ns, 'vehicle', [(0., 0., 0.), (1., 0., 0.)], self.t))
        return ns

    def test_start_needs_fresh_estimate_and_clock(self):
        self.assertFalse(ExplorationGuard().start(10.)[0])
        self.assertFalse(self.g.start(self.t+2)[0])
        self.start()
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))

    def test_initial_unplanned_waypoint_never_moves(self):
        self.start()
        ns = self.feed()
        self.g.tare_waypoint(ns, 'camera_init', (5., 0., 0.), self.t)
        self.assertIsNone(self.g.take_waypoint(self.t))
        self.assertFalse(self.g.local_path(ns, 'vehicle', [(0., 0., 0.), (1., 0., 0.)], self.t))
        self.assertFalse(self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t))
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))

    def test_lookahead_can_progress_without_goal_restarts(self):
        self.start()
        ns = self.plan()
        self.g.command(ns, 'vehicle', (1., -1., 2.), self.t)
        self.assertEqual(self.g.output(self.t), (.25, -.20, .50))
        ns = self.plan((4., 1., 0.))
        self.g.command(ns, 'vehicle', (.2, 0., .1), self.t)
        self.assertEqual(self.g.output(self.t), (.2, 0., .1))
        self.assertEqual(self.g.starts, 1)
        self.assertEqual(self.g.waypoints_forwarded, 2)

    def test_stop_latches_against_future_tare_output(self):
        self.start()
        self.plan()
        home = self.g.home
        self.g.stop()
        ns = self.feed(pos=(1., 0., 0.))
        self.assertFalse(self.g.tare_waypoint(ns, 'camera_init', (4., 0., 0.), self.t))
        self.assertFalse(self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t))
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertTrue(self.g.start(self.t)[0])
        self.assertEqual(self.g.home, home)
        self.assertIsNone(self.g.take_waypoint(self.t))

    def test_clock_rewind_cannot_be_cleared_by_start(self):
        self.start()
        self.plan()
        self.g.clock(1, self.t+.01)
        self.assertFalse(self.g.active)
        self.feed()
        self.assertFalse(self.g.start(self.t)[0])

    def test_planner_timeout_stops_despite_fresh_sensors(self):
        self.start()
        self.plan()
        for _ in range(32):
            self.feed()
            self.g.output(self.t)
        self.assertFalse(self.g.active)
        self.assertIn('TARE', self.g.reason)

    def test_cached_plan_cannot_extend_watchdog(self):
        self.start()
        ns = self.plan()
        for _ in range(31):
            self.feed()
            self.g.tare_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t)
            self.g.tare_waypoint(ns, 'camera_init', (3., 0., 0.), self.t)
            self.g.output(self.t)
        self.assertFalse(self.g.active)

    def test_sensor_failure_latches_zero(self):
        self.start()
        self.plan()
        for _ in range(12):
            self.feed(clouds=False)
            self.g.output(self.t)
        self.assertFalse(self.g.active)
        self.feed()
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))

    def test_out_of_bound_and_wrong_frame_are_rejected(self):
        self.start()
        ns = self.feed()
        self.assertFalse(self.g.tare_waypoint(ns, 'camera_init', (11., 0., 0.), self.t))
        self.assertFalse(self.g.active)
        self.start()
        self.assertFalse(self.g.tare_path(ns, 'map', [(0., 0., 0.), (1., 0., 0.)], self.t))
        self.assertFalse(self.g.active)

    def test_command_from_older_local_path_cannot_move(self):
        self.start()
        ns = self.plan()
        self.assertFalse(self.g.command(ns-1, 'vehicle', (.2, 0., 0.), self.t))
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t)
        for _ in range(6):
            self.feed()
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))

    def test_finish_means_returning_not_arrival(self):
        self.start()
        self.plan()
        self.feed(pos=(2., 0., 0.))
        self.g.finished(True)
        self.assertEqual(self.g.state, 'RETURNING_HOME')
        self.assertFalse(self.g.returned_home)
        self.g.output(self.t)
        self.assertTrue(self.g.active)

    def test_return_home_requires_stopped_motion_and_finish(self):
        self.start()
        self.plan()
        for _ in range(22):
            self.feed(pos=(.1, 0., 0.))
            self.g.output(self.t)
        self.assertTrue(self.g.active)
        self.assertFalse(self.g.returned_home)
        self.g.finished(True)
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertTrue(self.g.returned_home)
        self.assertEqual(self.g.state, 'RETURNED_HOME')
        self.assertFalse(self.g.start(self.t)[0])

    def test_parameters_validate_boundary_and_limits(self):
        with self.assertRaises(ValueError):
            GuardConfig(min_x=10., max_x=10.)
        with self.assertRaises(ValueError):
            GuardConfig(max_forward_m_s=2.)
        with self.assertRaises(ValueError):
            GuardConfig(min_y=float('nan'))


class GlobalPathFallbackTests(unittest.TestCase):
    setUp = GuardTests.setUp
    feed = GuardTests.feed
    start = GuardTests.start
    plan = GuardTests.plan

    def fallback(self, ns, local=()):
        self.g.tare_path(ns, 'camera_init', local, self.t)
        self.assertTrue(self.g.tare_global_path(ns, 'camera_init',
                        [(2., 0., 0.), (1., .2, 0.), (0., 0., 0.)], self.t))
        self.assertTrue(self.g.tare_waypoint(ns, 'camera_init', (0., 0., 0.), self.t))
        self.assertIsNotNone(self.g.take_waypoint(self.t))
        self.assertTrue(self.g.local_path(ns, 'vehicle', [(0., 0., 0.), (1., 0., 0.)], self.t))
        self.assertTrue(self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t))

    def test_empty_or_single_local_uses_real_full_global_path(self):
        for local in ([], [(2., 0., 0.)]):
            with self.subTest(local=local):
                self.setUp()
                self.start()
                ns = self.feed(pos=(2., 0., 0.))
                self.fallback(ns, local)
                self.assertEqual(self.g.output(self.t), (.2, 0., 0.))
                self.assertEqual(self.g.status(self.t)['selected_tare_path'], 'tare_global_path')
                self.assertEqual(self.g.status(self.t)['tare_global_path_poses'], 3)

    def test_callback_order_does_not_require_local_before_global_or_waypoint(self):
        for order in permutations(('local', 'global', 'waypoint')):
            with self.subTest(order=order):
                self.setUp()
                self.start()
                ns = self.feed(pos=(2., 0., 0.))
                callbacks = {
                    'local': lambda: self.g.tare_path(ns, 'camera_init', [(2., 0., 0.)], self.t),
                    'global': lambda: self.g.tare_global_path(ns, 'camera_init', [(2., 0., 0.), (0., 0., 0.)], self.t),
                    'waypoint': lambda: self.g.tare_waypoint(ns, 'camera_init', (0., 0., 0.), self.t),
                }
                for name in order:
                    callbacks[name]()
                    self.assertEqual(self.g.output(self.t), (0., 0., 0.))
                    self.assertTrue(self.g.active)
                self.assertIsNotNone(self.g.take_waypoint(self.t))
                self.assertEqual(self.g.plan_source, 'tare_global_path')

    def test_local_path_remains_preferred_when_both_routes_are_usable(self):
        self.start()
        ns = self.plan()
        self.g.tare_global_path(ns, 'camera_init', [(0., 0., 0.), (2., 1., 0.)], self.t)
        self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t)
        self.assertEqual(self.g.output(self.t), (.2, 0., 0.))
        self.assertEqual(self.g.plan_source, 'tare_path')

    def test_delayed_first_route_allows_next_waypoint_callback_within_startup(self):
        self.start()
        # TARE initialization may consume more than its steady-state 3s period.
        ns = self.feed(dt=4., pos=(2., 0., 0.))
        self.g.tare_global_path(ns, 'camera_init', [(2., 0., 0.), (0., 0., 0.)], self.t)
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertTrue(self.g.active)
        self.g.tare_waypoint(ns, 'camera_init', (0., 0., 0.), self.t)
        self.assertIsNotNone(self.g.take_waypoint(self.t))

    def test_path_heartbeat_without_first_waypoint_has_bounded_wait(self):
        self.start()
        self.feed(dt=4., pos=(2., 0., 0.))
        for _ in range(32):
            ns = self.feed(pos=(2., 0., 0.))
            self.g.tare_global_path(ns, 'camera_init', [(2., 0., 0.), (0., 0., 0.)], self.t)
            self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertFalse(self.g.active)

    def test_finish_flag_and_controller_path_cannot_replace_missing_tare_route(self):
        for global_points in ([], [(2., 0., 0.)]):
            with self.subTest(global_points=global_points):
                self.setUp()
                self.start()
                ns = self.feed(pos=(2., 0., 0.))
                self.g.finished(True)
                self.g.tare_path(ns, 'camera_init', [], self.t)
                self.g.tare_global_path(ns, 'camera_init', global_points, self.t)
                self.g.tare_waypoint(ns, 'camera_init', (0., 0., 0.), self.t)
                self.assertIsNone(self.g.take_waypoint(self.t))
                self.assertFalse(self.g.local_path(ns, 'vehicle', [(0., 0., 0.), (1., 0., 0.)], self.t))
                self.assertFalse(self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t))
                self.assertEqual(self.g.output(self.t), (0., 0., 0.))
                self.assertFalse(self.g.returned_home)

    def test_invalid_global_route_is_not_hidden_by_valid_local_route(self):
        for frame, points, stamp_delta in (
                ('map', [(0., 0., 0.), (1., 0., 0.)], 0),
                ('camera_init', [(0., 0., 0.), (float('nan'), 0., 0.)], 0),
                ('camera_init', [(0., 0., 0.), (11., 0., 0.)], 0),
                ('camera_init', [(0., 0., 0.), (1., 0., 0.)], 100000000)):
            with self.subTest(frame=frame, points=points, delta=stamp_delta):
                self.setUp()
                self.start()
                ns = self.plan()
                self.assertFalse(self.g.tare_global_path(ns+stamp_delta, frame, points, self.t))
                self.assertFalse(self.g.active)
                self.assertEqual(self.g.output(self.t), (0., 0., 0.))

    def test_cached_global_route_does_not_extend_three_second_watchdog(self):
        self.start()
        ns = self.feed(pos=(2., 0., 0.))
        self.fallback(ns)
        for _ in range(31):
            current = self.feed(pos=(2., 0., 0.))
            self.g.tare_path(current, 'camera_init', [], self.t)
            self.g.tare_waypoint(current, 'camera_init', (0., 0., 0.), self.t)
            self.g.tare_global_path(ns, 'camera_init', [(2., 0., 0.), (0., 0., 0.)], self.t)
            self.g.output(self.t)
        self.assertFalse(self.g.active)
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))

    def test_route_expiration_holds_zero_despite_fresh_short_paths_and_commands(self):
        self.start()
        ns = self.feed(pos=(2., 0., 0.))
        self.fallback(ns)
        for _ in range(32):
            current = self.feed(pos=(2., 0., 0.))
            self.g.tare_path(current, 'camera_init', [], self.t)
            self.g.tare_global_path(current, 'camera_init', [(2., 0., 0.)], self.t)
            self.g.tare_waypoint(current, 'camera_init', (0., 0., 0.), self.t)
            self.g.local_path(current, 'vehicle', [(0., 0., 0.), (1., 0., 0.)], self.t)
            self.g.command(current, 'vehicle', (.2, 0., 0.), self.t)
            self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertFalse(self.g.active)

    def test_late_mission_single_local_waits_for_global_without_immediate_latch(self):
        self.start()
        for _ in range(16):
            self.feed(dt=.9, pos=(2., 0., 0.))
            self.plan()
        self.assertGreater(self.t-self.g.started_wall, self.g.cfg.startup_timeout_sec)
        ns = self.feed(pos=(2., 0., 0.))
        self.g.tare_path(ns, 'camera_init', [(2., 0., 0.)], self.t)
        self.g.tare_waypoint(ns, 'camera_init', (0., 0., 0.), self.t)
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertTrue(self.g.active)
        self.fallback(ns, [(2., 0., 0.)])
        self.assertEqual(self.g.output(self.t), (.2, 0., 0.))

    def test_finish_still_requires_home_and_stable_motion_with_global_fallback(self):
        self.start()
        ns = self.feed(pos=(2., 0., 0.))
        self.fallback(ns)
        self.g.finished(True)
        self.assertEqual(self.g.output(self.t), (.2, 0., 0.))
        self.assertEqual(self.g.state, 'RETURNING_HOME')
        self.assertFalse(self.g.returned_home)
        for _ in range(22):
            ns = self.feed(pos=(.1, 0., 0.))
            if self.g.active:
                self.fallback(ns)
                self.g.output(self.t)
        self.assertTrue(self.g.returned_home)
        self.assertEqual(self.g.state, 'RETURNED_HOME')

    def test_global_route_cannot_rearm_explicit_stop(self):
        self.start()
        ns = self.feed(pos=(2., 0., 0.))
        self.fallback(ns)
        self.g.stop()
        ns = self.feed(pos=(2., 0., 0.))
        self.assertFalse(self.g.tare_global_path(ns, 'camera_init', [(2., 0., 0.), (0., 0., 0.)], self.t))
        self.assertIsNone(self.g.take_waypoint(self.t))
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertTrue(self.g.start(self.t)[0])
        self.assertEqual(self.g.global_plan_pose_count, 0)
        self.assertIsNone(self.g.plan_source)


class SimulationRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.g = ExplorationGuard(auto_resume_inputs=True, simulation_mode=True)
        self.t = 10.
        self.feed()
        self.assertTrue(self.g.start(self.t)[0])
        self.old_plan = self.plan()
        self.assertEqual(self.g.output(self.t), (.2, 0., 0.))

    def feed(self, dt=.1, clouds=True, pos=(0., 0., 0.)):
        self.t += dt
        ns = round(self.t*1e9)
        self.g.clock(ns, self.t)
        self.g.odometry(ns, 'camera_init', 'base_link', pos, (0., 0., 0., 1.), self.t)
        if clouds:
            for name in ('scan', 'terrain', 'terrain_ext'):
                self.g.cloud(name, ns, 'camera_init', 10, self.t)
        self.g.tracking(True, self.t)
        return ns

    def plan(self):
        ns = self.feed()
        self.assertTrue(self.g.tare_waypoint(ns, 'camera_init', (3., 0., 0.), self.t))
        self.assertTrue(self.g.tare_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t))
        self.assertIsNotNone(self.g.take_waypoint(self.t))
        self.assertTrue(self.g.local_path(ns, 'vehicle', [(0., 0., 0.), (1., 0., 0.)], self.t))
        self.assertTrue(self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t))
        return ns

    def pause(self):
        # Keep odometry, tracking and the simulation clock fresh; only clouds expire.
        for _ in range(11):
            self.feed(clouds=False)
            self.g.output(self.t)
        self.assertEqual(self.g.state, 'PAUSED_INPUTS')
        self.assertTrue(self.g.active)
        self.assertIsNone(self.g.path_ns)
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        return self.g.input_pause_stamp_ns

    def recover(self):
        for _ in range(8):
            self.feed()
            self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertEqual(self.g.state, 'WAITING_PLAN')
        self.assertEqual(self.g.input_resume_count, 1)

    def test_gap_stays_zero_and_recovery_needs_new_entire_plan_chain(self):
        fence = self.pause()
        # Incoming planner/command packets during a pause cannot accumulate authority.
        ns = self.feed()
        self.assertFalse(self.g.tare_waypoint(ns, 'camera_init', (3., 0., 0.), self.t))
        self.assertFalse(self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t))
        self.assertIsNone(self.g.take_waypoint(self.t))
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        during_pause_ns = ns
        self.recover()
        self.assertFalse(self.g.plan_valid)
        self.assertGreater(self.g.resume_after_stamp_ns, during_pause_ns)
        self.assertFalse(self.g.tare_path(during_pause_ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t))
        self.assertFalse(self.g.tare_waypoint(during_pause_ns, 'camera_init', (3., 0., 0.), self.t))
        self.assertFalse(self.g.tare_path(fence, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t))
        self.assertFalse(self.g.tare_waypoint(fence, 'camera_init', (3., 0., 0.), self.t))
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        ns = self.feed()
        self.g.tare_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t)
        self.g.tare_waypoint(ns, 'camera_init', (3., 0., 0.), self.t)
        self.assertIsNotNone(self.g.take_waypoint(self.t))
        self.assertFalse(self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t))
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertFalse(self.g.local_path(fence, 'vehicle', [(0., 0., 0.)], self.t))
        self.g.local_path(ns, 'vehicle', [(0., 0., 0.), (1., 0., 0.)], self.t)
        self.assertFalse(self.g.command(fence, 'vehicle', (.2, 0., 0.), self.t))
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t)
        self.assertEqual(self.g.output(self.t), (.2, 0., 0.))
        self.assertEqual(self.g.starts, 1)

    def test_fresh_dwell_resets_on_another_input_gap(self):
        self.pause()
        for _ in range(4):
            self.feed()
            self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertEqual(self.g.state, 'PAUSED_INPUTS')
        ns = self.feed()
        self.g.cloud('terrain_ext', ns, 'wrong_frame', 10, self.t)
        self.g.output(self.t)
        self.assertIsNone(self.g.input_fresh_since_wall)
        for _ in range(4):
            self.feed()
            self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertEqual(self.g.state, 'PAUSED_INPUTS')
        self.recover()

    def test_five_second_interruption_latches_even_after_recovery(self):
        self.pause()
        for _ in range(51):
            self.feed(clouds=False)
            self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertFalse(self.g.active)
        self.assertEqual(self.g.state, 'STOPPED')
        self.assertIn('5 seconds', self.g.reason)
        for _ in range(10):
            self.feed()
            self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertEqual(self.g.input_resume_count, 0)

    def test_explicit_stop_during_pause_never_auto_rearms(self):
        self.pause()
        self.g.stop()
        for _ in range(12):
            ns = self.feed()
            self.g.tare_waypoint(ns, 'camera_init', (3., 0., 0.), self.t)
            self.g.tare_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t)
            self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertFalse(self.g.active)
        self.assertEqual(self.g.state, 'STOPPED')
        self.assertEqual(self.g.input_resume_count, 0)

    def test_tracking_false_and_invalid_odom_are_not_recoverable_gaps(self):
        for fault in ('tracking', 'odometry', 'clock'):
            with self.subTest(fault=fault):
                self.setUp()
                self.pause()
                ns = self.feed()
                if fault == 'tracking':
                    self.g.tracking(False, self.t)
                elif fault == 'odometry':
                    self.g.odometry(ns, 'wrong_frame', 'base_link', (0.,0.,0.), (0.,0.,0.,1.), self.t)
                else:
                    self.g.clock(ns-1, self.t)
                self.assertFalse(self.g.active)
                for _ in range(8):
                    self.feed()
                    self.assertEqual(self.g.output(self.t), (0.,0.,0.))
                self.assertEqual(self.g.input_resume_count, 0)

    def test_finish_and_home_are_preserved_through_pause_and_resume(self):
        self.feed(pos=(2.,0.,0.))
        home = self.g.home
        self.g.finished(True)
        self.pause()
        self.g.finished(True)
        self.assertEqual(self.g.state, 'PAUSED_INPUTS')
        self.recover()
        self.assertTrue(self.g.finish_received)
        self.assertEqual(self.g.home, home)
        self.g.stop()
        self.assertTrue(self.g.start(self.t)[0])
        self.assertTrue(self.g.finish_received)
        self.assertEqual(self.g.home, home)

    def test_auto_resume_cannot_be_enabled_outside_simulation(self):
        with self.assertRaises(ValueError):
            ExplorationGuard(auto_resume_inputs=True)
        self.assertFalse(ExplorationGuard().auto_resume_inputs)

    def test_global_fallback_cannot_reuse_pre_recovery_route(self):
        ns = self.feed()
        self.g.tare_global_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t)
        self.pause()
        self.assertEqual(self.g.global_plan_pose_count, 0)
        self.assertNotIn('tare_global_path', self.g.times)
        self.assertFalse(self.g.tare_global_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t))
        self.recover()
        self.assertFalse(self.g.tare_global_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t))
        self.assertIsNone(self.g.take_waypoint(self.t))
        ns = self.feed()
        self.g.tare_path(ns, 'camera_init', [], self.t)
        self.g.tare_global_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t)
        self.g.tare_waypoint(ns, 'camera_init', (3., 0., 0.), self.t)
        self.assertIsNotNone(self.g.take_waypoint(self.t))
        self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t)
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.g.local_path(ns, 'vehicle', [(0., 0., 0.), (1., 0., 0.)], self.t)
        self.g.command(ns, 'vehicle', (.2, 0., 0.), self.t)
        self.assertEqual(self.g.output(self.t), (.2, 0., 0.))

    def test_recovery_delayed_first_global_allows_later_waypoint_callback(self):
        self.pause()
        self.recover()
        ns = self.feed(dt=4.)
        self.g.tare_global_path(ns, 'camera_init', [(0., 0., 0.), (3., 0., 0.)], self.t)
        self.assertEqual(self.g.output(self.t), (0., 0., 0.))
        self.assertTrue(self.g.active)
        self.g.tare_waypoint(ns, 'camera_init', (3., 0., 0.), self.t)
        self.assertIsNotNone(self.g.take_waypoint(self.t))


if __name__ == '__main__':
    unittest.main(verbosity=2)
