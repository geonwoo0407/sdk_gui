"""Hardware-free regression tests for restored GUI cosine playback."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from continuous_trajectory import ContinuousTrajectory

source = ast.parse(Path(__file__).with_name('sdk_gui.py').read_text())
method_names = {
    'normalize_angles', 'shortest_angle_delta',
    'anim_step', 'scrub_timeline', 'frame_motion_progress', 'timeline_frame_angles',
    'interpolate_angle_shortest', 'gate_timeline_on_frame_arrival',
    'fail_timed_playback',
    'map_timeline_time', 'playback_time_ms',
    'next_playback_cycle_end', 'restart_playback_cycle',
}
methods = [node for cls in source.body if isinstance(cls, ast.ClassDef)
           for node in cls.body if isinstance(node, ast.FunctionDef)
           and node.name in method_names]
constants = [node for node in source.body if isinstance(node, ast.Assign)
             and all(isinstance(t, ast.Name) and t.id.isupper() for t in node.targets)]
namespace = {
    'ContinuousTrajectory': ContinuousTrajectory,
    'math': math, 'time': SimpleNamespace(perf_counter=lambda: 0.0),
    'TimelineBlockWidget': object,
}
# Only literal configuration values are required; avoid GUI/SDK imports entirely.
for node in constants:
    try:
        exec(compile(ast.Module(body=[node], type_ignores=[]), '<constants>', 'exec'), namespace)
    except (NameError, TypeError):
        pass
exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Player', bases=[], keywords=[],
    body=methods, decorator_list=[])], type_ignores=[])), '<gui methods>', 'exec'), namespace)
Player = namespace['Player']


def player():
    p = Player()
    p.online_joints = {0, 1}
    p.time_based_profile_ids = {0, 1}
    p.playback_repeat_current = 1
    p.playback_speed = 1.0
    p.active_timed_frame_token = None
    p.feedback_angles = {0: 0.0, 1: 0.0}
    p.calls = []
    p.errors = []
    p.rendered = []
    p.stops = []
    p.holds = []
    p.set_time_profile_duration = lambda ids, duration, accel_ms: p.calls.append(('profile', duration, accel_ms)) or True
    p.write_goal_positions = lambda angles, **kwargs: p.calls.append(('goal', dict(angles))) or True
    p.read_feedback_and_report_fk = lambda: True
    namespace['QMessageBox'] = SimpleNamespace(
        critical=lambda parent, title, message: p.errors.append(message))
    p.hold_timed_motion_at_feedback = lambda: p.holds.append(True)
    p.anim_timer = SimpleNamespace(stop=lambda: None)
    p.set_tracking_target_angles = lambda angles: None
    p.is_playing = p.execute_on_real_robot = p.robot_sync_enabled = True
    p.playback_repeat_target = 1
    p.anim_start_time = 0.0
    p.timed_gate_frame = None
    p.last_playback_ui_time = 0.0
    p.last_timed_feedback_time = 0.0
    p.current_timeline_ms = 0
    p.playback_context = 'motion'
    p.SCALE = 1.0
    p.start_angles = {0: 0.0, 1: 0.0}
    p.joints = p.start_angles.copy()
    p.update_3d_robot = lambda angles: p.rendered.append(dict(angles))
    p.update_playback_buttons = lambda: None
    p.timeline_container = SimpleNamespace(
        set_playhead=lambda *args: None, findChildren=lambda *args: [])
    p.timeline_scroll = SimpleNamespace(
        ensureVisible=lambda *args: None, height=lambda: 100,
        horizontalScrollBar=lambda: SimpleNamespace(setValue=lambda value: None))

    def stop(keep_timed_goal=False):
        p.stops.append(keep_timed_goal)
        p.is_playing = False

    p.stop_motion_sequence = stop
    set_frames(p, [frame()])
    return p


def frame(name='regular', start=0, duration=100, angles=None):
    return dict(name=name, start_ms=start, time_ms=duration,
                angles=angles or {'0': 30.0}, lift_early_arrival=True)


def set_frames(p, frames):
    p.motion_sequence = frames
    p.max_seq_ms = max(f['start_ms'] + f['time_ms'] for f in frames)
    p.anim_duration = p.max_seq_ms / 1000.0


def tick(p, seconds):
    namespace['time'].perf_counter = lambda: seconds
    p.anim_step()


class PlaybackTests(unittest.TestCase):
    def setUp(self):
        namespace['time'].perf_counter = lambda: 0.0

    def test_cosine_intermediate_goals_every_five_ms(self):
        p = player()
        self.assertEqual(namespace['CONTROL_INTERVAL_MS'], 5)
        for i in range(20):
            tick(p, i * 0.005)
        self.assertEqual(len(p.calls), 20)
        for i, (kind, angles) in enumerate(p.calls):
            self.assertEqual(kind, 'goal')
            expected = 30 * (0.5 - 0.5 * math.cos(math.pi * i / 20))
            self.assertAlmostEqual(angles[0], expected)

    def test_landing_uses_same_cosine_with_playback_speed(self):
        p = player()
        set_frames(p, [frame('[착지]')])
        p.playback_speed = 0.5
        tick(p, 0.0)
        tick(p, 0.05)
        self.assertAlmostEqual(p.calls[-1][1][0], 30 * (1 - math.cos(math.pi / 4)) / 2)
        self.assertTrue(all(kind == 'goal' for kind, _ in p.calls))

    def test_lift_reaches_at_eighty_percent_and_holds(self):
        p = player()
        set_frames(p, [frame('오들')])
        tick(p, 0.0)
        tick(p, 0.08)
        self.assertEqual(p.calls[-1], ('goal', {0: 30.0}))
        tick(p, 0.095)
        self.assertEqual(p.calls[-1], ('goal', {0: 30.0}))
        self.assertTrue(p.is_playing)

    def test_lift_without_option_uses_entire_duration(self):
        p = player()
        f = frame('오들')
        f['lift_early_arrival'] = False
        set_frames(p, [f])
        tick(p, 0.08)
        self.assertLess(p.calls[-1][1][0], 30.0)

    def test_next_frame_starts_on_time_without_arrival(self):
        p = player()
        set_frames(p, [frame(), frame(start=100, angles={'0': 60.0})])
        tick(p, 0.0)
        tick(p, 0.05)
        self.assertEqual(len(p.calls), 2)
        tick(p, 0.1)
        self.assertEqual(p.calls[-1], ('goal', {0: 30.0}))
        self.assertEqual(p.current_timeline_ms, 100)
        self.assertEqual(p.anim_start_time, 0.0)
        self.assertEqual(p.errors, [])
        tick(p, 0.15)
        self.assertAlmostEqual(p.calls[-1][1][0], 45.0)

    def test_no_three_second_timeout_when_feedback_fails(self):
        p = player()
        set_frames(p, [frame(duration=5000)])
        p.read_feedback_and_report_fk = lambda: False
        tick(p, 0.0)
        tick(p, 3.1)
        self.assertTrue(p.is_playing)
        self.assertEqual(p.current_timeline_ms, 3100)
        self.assertEqual(p.errors, [])
        self.assertEqual(len(p.calls), 2)

    def test_final_goal_sent_before_finishing_without_settling(self):
        p = player()
        p.read_feedback_and_report_fk = lambda: False
        tick(p, 0.0)
        tick(p, 0.099)
        self.assertTrue(p.is_playing)
        tick(p, 0.1)
        self.assertFalse(p.is_playing)
        self.assertEqual(p.stops, [True])
        self.assertEqual(p.errors, [])
        self.assertEqual(p.calls[-1], ('goal', {0: 30.0}))
        self.assertEqual(p.holds, [])

    def test_screen_shows_timeline_angles(self):
        p = player()
        set_frames(p, [frame(name='regular')])
        p.feedback_angles = {0: -90.0, 1: 50.0}
        tick(p, 0.0)
        tick(p, 0.05)
        self.assertEqual(len(p.rendered), 1)
        self.assertAlmostEqual(p.rendered[0][0], 15.0)
        self.assertEqual(p.calls[-1][1], p.rendered[0])

    def test_repetition_starts_without_arrival_wait(self):
        p = player()
        p.playback_repeat_target = 2
        tick(p, 0.0)
        tick(p, 0.1)
        self.assertEqual(p.playback_repeat_current, 2)
        self.assertTrue(p.is_playing)
        self.assertEqual(p.stops, [])
        tick(p, 0.1)
        self.assertEqual(p.calls[-1], ('goal', {0: 30.0}))
        tick(p, 0.2)
        self.assertEqual(p.stops, [True])

    def test_no_online_targets_stops_playback(self):
        p = player()
        p.online_joints = set()
        tick(p, 0.0)
        self.assertEqual(p.calls, [])
        self.assertFalse(p.is_playing)
        self.assertEqual(len(p.errors), 1)

    def test_first_frame_interpolates_from_initial_pose(self):
        p = player()
        p.start_angles[0] = 10.0
        tick(p, 0.0)
        self.assertEqual(p.calls[-1], ('goal', {0: 10.0}))
        tick(p, 0.05)
        self.assertAlmostEqual(p.calls[-1][1][0], 20.0)

    def test_late_boundary_sends_previous_endpoint_without_resetting_clock(self):
        p = player()
        set_frames(p, [frame(), frame(start=100, angles={'0': 60.0})])
        tick(p, 0.0)
        tick(p, 0.125)
        self.assertEqual(p.calls[-2], ('goal', {0: 30.0}))
        self.assertEqual(p.current_timeline_ms, 100)
        self.assertEqual(p.anim_start_time, 0.0)
        tick(p, 0.13)
        self.assertEqual(p.current_timeline_ms, 130)
        self.assertGreater(p.calls[-1][1][0], 30.0)

    def test_goal_send_failure_stops_playback(self):
        p = player()
        p.write_goal_positions = lambda *args, **kwargs: False
        tick(p, 0.0)
        self.assertEqual(p.calls, [])
        self.assertEqual(len(p.errors), 1)
        self.assertFalse(p.is_playing)
        self.assertEqual(p.holds, [True])


if __name__ == '__main__':
    unittest.main()
