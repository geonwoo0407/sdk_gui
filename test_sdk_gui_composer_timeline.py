"""Verify composer timing and real Qt rendering without opening a motor port."""
import ast
import copy
import contextlib
import io
import json
import hashlib
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
import uuid
from continuous_trajectory import ContinuousTrajectory
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtTest import QTest
import test_sdk_gui_timed_playback as playback


# The application constructor opens hardware. Load its methods and timeline
# widgets, then build only the composer page with real Qt controls.
source = ast.parse(Path(__file__).with_name('sdk_gui.py').read_text())
namespace = {'copy': copy, 'json': json, 'math': math, 'hashlib': hashlib, 'os': os, 'uuid': uuid}
namespace['ContinuousTrajectory'] = ContinuousTrajectory
for module in (QtCore, QtGui, QtWidgets):
    namespace.update(vars(module))
nodes = []
for node in source.body:
    if isinstance(node, ast.Assign):
        if all(isinstance(target, ast.Name) and target.id.isupper() for target in node.targets):
            try:
                exec(compile(ast.Module(body=[node], type_ignores=[]), '<constants>', 'exec'), namespace)
            except (NameError, TypeError):
                pass
    elif isinstance(node, ast.FunctionDef) and node.name.startswith('draw_timeline_'):
        nodes.append(node)
    elif isinstance(node, ast.ClassDef):
        if node.name in {'TimelineContainer', 'SequenceTimelineBlockWidget', 'SequenceTimelineContainer'}:
            nodes.append(node)
        elif node.name == 'SDKMotionEditor':
            node.name = 'Editor'
            node.body = [method for method in node.body if method.name != '__init__']
            nodes.append(node)
exec(compile(ast.Module(body=nodes, type_ignores=[]), '<composer paths>', 'exec'), namespace)


def source_sequence(speed=0.9):
    return {
        'name': '후진실전-2(3회)', 'playback_speed': speed, 'repeat_count': 3,
        'frames': [
            dict(name=name, start_ms=start, time_ms=duration,
                 angles={'17': -12.392578125}, lift_early_arrival=True)
            for name, start, duration in (
                ('미세후진왼들3', 60, 55), ('미세후진왼뒤', 115, 120),
                ('미세후진오들3', 292, 55), ('미세후진오뒤', 347, 100),
            )
        ],
    }


def timings(frames):
    return [(frame['start_ms'], frame['time_ms']) for frame in frames]


class ComposerTimelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.editor = p = namespace['Editor']()
        self.addCleanup(p.close)
        p.SCALE = 2.5
        p.max_seq_ms = 5000
        p.frames = []
        p.saved_sequences = [source_sequence()]
        p.playback_context = 'motion'
        p.is_playing = p.is_paused = False
        p.playback_paused_by_button = False
        p.current_timeline_ms = 0
        self.page = QtWidgets.QWidget()
        self.addCleanup(self.page.close)
        p.init_sequence_composer_tab(self.page)
        p.add_sequence_to_composer(0)

    def tearDown(self):
        # Flush deferred block rebuilds while their page still exists.
        self.app.processEvents()

    def test_screenshot_timings_match_motion_at_every_saved_speed(self):
        p = self.editor
        expected = [(start + repetition * 447, duration)
                    for repetition in range(3)
                    for start, duration in ((60, 55), (115, 120), (292, 55), (347, 100))]
        for speed in (0.1, 0.9, 0.95, 1.0, 5.0):
            with self.subTest(speed=speed):
                p.saved_sequences[0]['playback_speed'] = speed
                p.refresh_composer_source_list()
                frames, duration = p.build_composed_sequence_frames()
                entry = p.sequence_composer_entries[0]
                self.assertEqual(timings(frames), expected)
                self.assertEqual(timings(entry['frames']), expected)
                self.assertEqual(duration, 1341)
                self.assertEqual(entry['time_ms'], duration)
                self.assertEqual(entry['leading_wait_ms'], 60)
                self.assertEqual(entry['playback_speed'], speed)
                segments = p.composer_timing_segments()
                self.assertAlmostEqual(segments[-1][3], 1341 / speed)
                self.assertIn('1341ms', p.lbl_composer_meta.text())

    def test_mixed_entries_keep_gaps_repeats_metadata_and_absolute_offsets(self):
        p = self.editor
        original = copy.deepcopy(p.saved_sequences)
        first = p.sequence_composer_entries[0]
        p.set_composer_entry_repeat(first, 2)
        p.set_composer_entry_gap(first, 13)
        p.frames = [dict(name='single', frame_id='single', time_ms=10, angles={'17': 1.25})]
        p.add_frame_to_composer('single')
        single = p.sequence_composer_entries[-1]
        p.set_composer_entry_gap(single, 7)
        p.set_composer_entry_repeat(single, 2)
        p.add_sequence_to_composer(0)
        last = p.sequence_composer_entries[-1]
        p.set_composer_entry_repeat(last, 1)
        frames, duration = p.build_composed_sequence_frames()
        self.assertEqual(frames[0]['start_ms'], 73)
        self.assertEqual(single['start_ms'], 907)
        self.assertEqual(timings(frames[8:10]), [(914, 10), (924, 10)])
        self.assertEqual(last['start_ms'], 934)
        self.assertEqual(timings(frames[10:]), [(994, 55), (1049, 120), (1226, 55), (1281, 100)])
        self.assertEqual(duration, 1381)
        self.assertEqual(duration, p.composer_total_duration())
        self.assertEqual(p.saved_sequences, original)
        self.assertEqual(frames[0]['angles'], {'17': -12.392578125})
        self.assertTrue(frames[0]['lift_early_arrival'])
        frames[0]['angles']['17'] = 0
        self.assertEqual(p.saved_sequences, original)

    def test_export_bakes_source_speed_once_and_preserves_global_speed(self):
        p = self.editor
        p.motion_sequence = [{'name': 'original motion'}]
        p.max_seq_ms = 5000
        p.spin_composer_speed.setValue(0.9)
        self.assertTrue(p.activate_composer_playback())
        self.assertEqual(p.max_seq_ms, 1341)
        self.assertEqual(timings(p.motion_sequence[:4]), [(60, 55), (115, 120), (292, 55), (347, 100)])
        p.play_keyframe_sequence = Mock()
        p.play_composed_sequence()
        self.assertEqual(p.playback_speed, 0.9)
        with TemporaryDirectory() as directory:
            output = Path(directory) / 'composed.json'
            with patch.object(QtWidgets.QFileDialog, 'getSaveFileName', return_value=(str(output), '')), \
                    patch.object(QtWidgets.QMessageBox, 'information'):
                p.export_composed_sequence_json()
            exported = json.loads(output.read_text())
        self.assertEqual(exported['max_seq_ms'], 1490)
        self.assertEqual(exported['playback_speed'], 0.9)
        self.assertEqual(timings(exported['frames'][:4]), [(67, 61), (128, 133), (324, 62), (386, 111)])
        for index, frame in enumerate(exported['frames']):
            original = p.motion_sequence[index]
            self.assertEqual(frame['start_ms'], round(original['start_ms'] / 0.9))
            self.assertEqual(frame['start_ms'] + frame['time_ms'],
                             round((original['start_ms'] + original['time_ms']) / 0.9))
            self.assertEqual(frame['angles'], original['angles'])
        # A nested composition applies its own saved global speed once.
        p.saved_sequences.append(dict(exported, name='composed'))
        frames, duration = p.composer_entry_frames({'sequence_idx': 1})
        self.assertEqual(duration, 1490)
        self.assertEqual(frames, exported['frames'])
        p.sequence_composer_entries = []
        p.playback_context = 'motion'
        p.add_sequence_to_composer(1)
        nested, duration = p.build_composed_sequence_frames(apply_source_speed=True)
        self.assertEqual(duration, round(1490 / 0.9))
        self.assertEqual(nested[0]['start_ms'], round(67 / 0.9))

    def test_mixed_source_speeds_match_standalone_motor_goals(self):
        p = self.editor
        first = dict(name='first', frames=[playback.frame(start=20, angles={'0': 0.0})],
                     playback_speed=2.0, repeat_count=1)
        source_frames = [playback.frame(start=20), playback.frame(start=120, duration=80, angles={'0': 60.0})]
        for speed, global_speed in ((0.5, 1.0), (0.9, 1.0), (1.25, 1.0), (0.5, 2.0)):
            with self.subTest(speed=speed, global_speed=global_speed):
                p.saved_sequences = [first, dict(name='turn', frames=source_frames,
                                               playback_speed=speed, repeat_count=1)]
                p.sequence_composer_entries = []
                p.add_sequence_to_composer(0)
                p.set_composer_entry_gap(p.sequence_composer_entries[0], 40)
                p.add_sequence_to_composer(1)
                p.set_composer_entry_gap(p.sequence_composer_entries[1], 30)
                standalone, combined = playback.player(), playback.player()
                playback.set_frames(standalone, source_frames)
                frames, _ = p.build_composed_sequence_frames()
                playback.set_frames(combined, frames)
                standalone.playback_speed = speed * global_speed
                combined.playback_speed = global_speed
                combined.playback_context = 'composer'
                combined.composer_playback_segments = p.composer_timing_segments()
                combined.COMPOSER_SCALE = p.COMPOSER_SCALE
                combined.composer_timeline_container = combined.timeline_container
                combined.composer_timeline_scroll = combined.timeline_scroll
                # The first entry and explicit gaps take 130ms before global speed.
                offset_ticks = round(130 / global_speed / 5)
                for tick in range(offset_ticks):
                    playback.tick(combined, tick * 0.005)
                ticks = math.ceil(200 / (speed * global_speed) / 5)
                for tick in range(ticks + 1):
                    playback.tick(standalone, tick * 0.005)
                    playback.tick(combined, (tick + offset_ticks) * 0.005)
                    self.assertEqual(combined.calls[-1], standalone.calls[-1])
                self.assertFalse(combined.is_playing)

    def test_pause_resume_uses_source_speed_clock(self):
        p = self.editor
        p.motion_sequence = []
        p.max_seq_ms = 5000
        p.activate_composer_playback()
        p.is_playing = True
        p.playback_speed = 1.0
        p.anim_start_time = 0.0
        p.hold_timed_motion_at_feedback = Mock()
        p.update_playback_buttons = Mock()
        p.anim_timer = Mock()
        p.robot_scrub_timer = Mock()
        p.start_angles = p.joints = {17: 0.0}
        with patch.dict(namespace, time=SimpleNamespace(perf_counter=lambda: 0.5)):
            p.pause_motion_sequence()
        self.assertEqual(p.current_timeline_ms, 450)
        with patch.dict(namespace, time=SimpleNamespace(perf_counter=lambda: 10.0)):
            p.start_motion_playback(start_ms=p.current_timeline_ms)
        self.assertAlmostEqual(p.anim_start_time, 9.5)
        self.assertAlmostEqual(p.playback_time_ms(600, to_timeline=True), 540)

    def test_repeated_motion_matches_composer_with_feedback_latency(self):
        p = self.editor

        def run(frames, speed, repeat, segments=None, feedback_delay=0):
            player = playback.player()
            clock = [0.0]
            trace = []
            playback.set_frames(player, copy.deepcopy(frames))
            player.playback_speed = speed
            player.playback_repeat_target = repeat
            if segments is not None:
                player.playback_context = 'composer'
                player.composer_playback_segments = segments
                player.COMPOSER_SCALE = p.COMPOSER_SCALE
                player.composer_timeline_container = player.timeline_container
                player.composer_timeline_scroll = player.timeline_scroll

            def feedback():
                clock[0] += feedback_delay
                return True

            def goal(angles, **kwargs):
                trace.append((round(clock[0], 6), dict(angles)))
                return True

            player.read_feedback_and_report_fk = feedback
            player.write_goal_positions = goal
            with patch.dict(playback.namespace, time=SimpleNamespace(perf_counter=lambda: clock[0])), \
                    contextlib.redirect_stdout(io.StringIO()):
                while player.is_playing and clock[0] < 30:
                    player.anim_step()
                    clock[0] += 0.005
            self.assertFalse(player.is_playing)
            return clock[0], trace

        for leading_wait in (0, 90):
            for delay in (0, 0.005, 0.015, 0.04):
                for global_speed in (1.0, 2.0):
                    with self.subTest(wait=leading_wait, feedback=delay, global_speed=global_speed):
                        source_frames = [
                            playback.frame('제우오들', start=leading_wait, duration=60, angles={'0': 30.0}),
                            playback.frame('오뒤', start=leading_wait + 60, duration=130, angles={'0': 0.0}),
                        ]
                        p.saved_sequences = [dict(name='turn', frames=source_frames,
                                                  playback_speed=0.9, repeat_count=7)]
                        p.sequence_composer_entries = []
                        p.add_sequence_to_composer(0)
                        combined, _ = p.build_composed_sequence_frames()
                        self.assertEqual(sum(bool(f.get('playback_cycle_end')) for f in combined), 7)
                        standalone_time, standalone_trace = run(
                            source_frames, 0.9 * global_speed, 14, feedback_delay=delay,
                        )
                        composer_time, composer_trace = run(
                            combined, global_speed, 2, p.composer_timing_segments(), delay,
                        )
                        self.assertAlmostEqual(composer_time, standalone_time)
                        self.assertEqual(len(composer_trace), len(standalone_trace))
                        for (ct, ca), (st, sa) in zip(composer_trace, standalone_trace):
                            self.assertEqual(ct, st)
                            self.assertEqual(ca.keys(), sa.keys())
                            for joint in ca:
                                self.assertAlmostEqual(ca[joint], sa[joint])

    def test_save_preserves_speed_and_original_lift_option(self):
        p = self.editor
        p.saved_sequences[0]['frames'][0]['lift_early_arrival'] = False
        p.refresh_sequence_list = Mock()
        p.save_persistent_state = Mock()
        with patch.object(QtWidgets.QInputDialog, 'getText', return_value=('saved', True)), \
                patch.object(QtWidgets.QMessageBox, 'information'):
            p.save_composed_sequence()
        saved = p.saved_sequences[-1]
        self.assertEqual(saved['max_seq_ms'], 1490)
        self.assertEqual(saved['playback_speed'], 1.0)
        self.assertFalse(saved['frames'][0]['lift_early_arrival'])
        self.assertEqual(timings(saved['frames'][:2]), [(67, 61), (128, 133)])
        self.assertEqual(sum(bool(f.get('playback_cycle_end')) for f in saved['frames']), 3)
        reloaded = [p.normalize_frame_data(f) for f in saved['frames']]
        self.assertEqual([f.get('playback_cycle_end') for f in reloaded],
                         [f.get('playback_cycle_end') for f in saved['frames']])

    def test_fast_saved_composition_keeps_sub_ten_ms_frames(self):
        p = self.editor
        p.saved_sequences = [dict(name='fast', playback_speed=5.0, repeat_count=1,
                                  frames=[playback.frame(start=10, duration=10)])]
        p.sequence_composer_entries = []
        p.add_sequence_to_composer(0)
        frames, duration = p.build_composed_sequence_frames(apply_source_speed=True)
        self.assertEqual(timings(frames), [(2, 2)])
        self.assertEqual(duration, 4)
        self.assertAlmostEqual(p.frame_motion_progress(frames[0], 3), 0.5)
        self.assertEqual(p.frame_motion_progress(frames[0], 4), 1.0)
        p.saved_sequences.append(dict(name='saved', frames=frames, playback_speed=1.0))
        nested, nested_duration = p.composer_entry_frames({'sequence_idx': 1})
        self.assertEqual(timings(nested), [(2, 2)])
        self.assertEqual(nested_duration, 4)

    def test_play_after_scrubbing_uses_composer_global_speed(self):
        p = self.editor
        p.motion_sequence = []
        p.max_seq_ms = 5000
        p.activate_composer_playback()
        p.is_paused = True
        p.playback_paused_by_button = False
        p.playback_speed = 5.0
        p.spin_composer_speed.setValue(1.0)
        p.play_keyframe_sequence = Mock()
        p.play_composed_sequence()
        self.assertEqual(p.playback_speed, 1.0)
        self.assertAlmostEqual(p.playback_time_ms(100, to_timeline=True), 90)

    def test_known_old_composition_is_repaired_on_restart_only_when_unchanged(self):
        p = self.editor
        old_frames = copy.deepcopy(p.saved_sequences[0]['frames'])
        old_sequence = dict(p.saved_sequences[0], sequence_id='old-composition', playback_speed=1.0)
        rule = {
            'sequence_id': 'old-composition', 'playback_speed': 1.0,
            'repeat_count': old_sequence['repeat_count'],
            'before': p.sequence_timing_fingerprint(old_frames),
            'timings': [[f['start_ms'] * 2, f['time_ms'] * 2, i == len(old_frames) - 1]
                        for i, f in enumerate(old_frames)],
        }
        with TemporaryDirectory() as directory:
            p.state_file_path = str(Path(directory) / 'sdk_gui_state.json')
            Path(directory, namespace['TIMING_REPAIRS_FILENAME']).write_text(json.dumps([rule]))
            state = dict(frames=[], saved_sequences=[old_sequence], motion_sequence=old_frames,
                         loaded_sequence_id='old-composition', max_seq_ms=447)
            Path(p.state_file_path).write_text(json.dumps(state))
            for name in ('rebuild_frame_list_ui', 'refresh_sequence_list', 'refresh_timeline_ui',
                         'refresh_loaded_sequence_indicator'):
                setattr(p, name, Mock())
            p.load_persistent_state()
            self.assertEqual(p.max_seq_ms, 894)
            self.assertEqual(timings(p.motion_sequence), [(120, 110), (230, 240), (584, 110), (694, 200)])
            self.assertEqual(p.motion_sequence, p.saved_sequences[0]['frames'])
            self.assertTrue(p.motion_sequence[-1]['playback_cycle_end'])
            corrected = copy.deepcopy(p.saved_sequences)
            self.assertEqual(p.repair_known_composer_timings(), [])
            self.assertEqual(p.saved_sequences, corrected)
            # A stale GUI may write the original file again; loading repairs it again.
            p.load_persistent_state()
            self.assertEqual(p.saved_sequences, corrected)
            # A user's new pose edit is not eligible for the historical correction.
            changed = copy.deepcopy(old_sequence)
            changed['frames'][0]['angles']['17'] = 123
            p.saved_sequences = [changed]
            self.assertEqual(p.repair_known_composer_timings(), [])
            self.assertEqual(p.saved_sequences, [changed])

    def test_motion_page_fresh_play_after_scrub_uses_its_current_speed(self):
        p = self.editor
        p.playback_context = 'motion'
        p.is_paused = True
        p.playback_paused_by_button = False
        p.playback_speed = 0.5
        p.spin_motion_speed = SimpleNamespace(value=lambda: 0.9)
        p.spin_motion_repeat = SimpleNamespace(value=lambda: 7)
        p.play_keyframe_sequence = Mock()
        p.play_motion_page_sequence()
        self.assertEqual(p.playback_speed, 0.9)
        self.assertEqual(p.playback_repeat_target, 7)

    def test_zoom_button_uses_spinbox_value_and_scrubbing_stays_on_axis(self):
        p = self.editor
        self.assertEqual(p.COMPOSER_SCALE, p.SCALE)
        p.motion_sequence = []
        p.max_seq_ms = 5000
        p.scrub_timeline = Mock()
        for zoom in (25, 100, 250, 400):
            p.spin_composer_zoom.setValue(zoom)
            QTest.mouseClick(p.btn_apply_composer_zoom, QtCore.Qt.LeftButton)
            self.assertEqual(p.COMPOSER_SCALE, zoom / 100)
            p.composer_timeline_container.scrub(200 * p.COMPOSER_SCALE)
            p.scrub_timeline.assert_called_with(200)
        p.composer_timeline_container.scrub(999999)
        p.scrub_timeline.assert_called_with(1341)

    def test_rendered_paths_and_header_bounds_share_absolute_time_coordinates(self):
        p = self.editor
        p.frames = [dict(name='a very long name', frame_id='short', time_ms=10, angles={})]
        p.add_frame_to_composer('short')
        p.add_sequence_to_composer(0)
        p.spin_composer_zoom.setValue(150)
        p.btn_apply_composer_zoom.click()
        container = p.composer_timeline_container
        container.resize(p.composer_timeline_width(), 290)
        container.show()
        self.app.processEvents()
        for block, entry in zip(container.findChildren(namespace['SequenceTimelineBlockWidget']),
                                p.sequence_composer_entries):
            self.assertEqual(block.x(), int(entry['start_ms'] * 1.5))
            self.assertEqual(block.x() + block.width(), int((entry['start_ms'] + entry['time_ms']) * 1.5))
        draw_frames = namespace['draw_timeline_frames']
        draw_ruler = namespace['draw_timeline_ruler']
        paths, rulers = [], []

        def record_frames(painter, frames, *args, **kwargs):
            paths.append(copy.deepcopy(frames))
            return draw_frames(painter, frames, *args, **kwargs)

        def record_ruler(painter, duration, scale, **kwargs):
            rulers.append((duration, scale))
            return draw_ruler(painter, duration, scale, **kwargs)

        with patch.dict(namespace, draw_timeline_frames=record_frames,
                        draw_timeline_ruler=record_ruler):
            container.grab()
            expected, duration = p.build_composed_sequence_frames()
            self.assertEqual(timings(paths[-1]), timings(expected))
            self.assertEqual(rulers[-1], (duration, 1.5))

    def test_motion_and_composer_render_identical_time_axis_and_frame_positions(self):
        p = self.editor
        p.set_composer_entry_repeat(p.sequence_composer_entries[0], 1)
        p.motion_sequence = p.saved_sequences[0]['frames']
        p.max_seq_ms = 447
        motion = namespace['TimelineContainer'](p)
        self.addCleanup(motion.close)
        motion.resize(1200, 250)
        motion.show()
        composer = p.composer_timeline_container
        composer.resize(1200, 290)
        composer.show()
        self.app.processEvents()
        motion_image = motion.grab().toImage()
        composer_image = composer.grab().toImage()
        for x in range(int(447 * p.SCALE) + 1):
            # The header is taller in the composer; horizontal time is identical.
            self.assertEqual(motion_image.pixel(x, 165), composer_image.pixel(x, 190))
            self.assertEqual(motion_image.pixel(x, 20), composer_image.pixel(x, 20))


if __name__ == '__main__':
    unittest.main()
