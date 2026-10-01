"""Exercise real Qt angle controls and JSON exports without opening a motor port."""
import ast
import copy
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import uuid

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import (
    QApplication, QDoubleSpinBox, QSlider, QWidget, QVBoxLayout, QLineEdit,
    QPushButton, QStyle, QStyleOptionSpinBox,
)

import test_sdk_gui_timed_playback as playback


# Load just the controls/data paths: SDKMotionEditor's constructor opens hardware.
source = ast.parse(Path(__file__).with_name('sdk_gui.py').read_text())
method_names = {
    'angle_to_slider_value', 'dxl_position_to_angle', 'angle_to_dxl_position',
    'update_joint_display', 'sync_values', 'refresh_live_joint_angles',
    'set_joint_connection_ui',
    'normalize_angles', 'normalize_frame_data', 'current_persistent_state_data',
    'add_frame', 'frames_for_json_export', 'repair_legacy_sequence_timing',
    'export_motion_json', 'export_motion_collection', 'export_all_motions_json',
    'export_checked_motions_json', 'export_composed_sequence_json',
}
namespace = {
    'QDoubleSpinBox': QDoubleSpinBox, 'QSlider': QSlider,
    'copy': copy, 'json': json, 'uuid': uuid,
    'QListWidgetItem': lambda parent: Mock(),
    'FrameItemWidget': lambda data, parent: Mock(),
    'QMessageBox': SimpleNamespace(information=Mock(), warning=Mock()),
}
for node in source.body:
    if (isinstance(node, ast.Assign)
            and all(isinstance(t, ast.Name) and t.id.isupper() for t in node.targets)):
        try:
            exec(compile(ast.Module(body=[node], type_ignores=[]), '<constants>', 'exec'), namespace)
        except (NameError, TypeError):
            pass
classes = [node for node in source.body if isinstance(node, ast.ClassDef)
           and node.name in {'NoWheelAngleSpinBox', 'NoWheelSlider'}]
methods = [node for cls in source.body if isinstance(cls, ast.ClassDef)
           and cls.name == 'SDKMotionEditor' for node in cls.body
           if isinstance(node, ast.FunctionDef) and node.name in method_names]
classes.append(ast.ClassDef(name='Editor', bases=[], keywords=[], body=methods, decorator_list=[]))
exec(compile(ast.fix_missing_locations(ast.Module(body=classes, type_ignores=[])),
             '<gui precision paths>', 'exec'), namespace)


class PrecisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.editor = p = namespace['Editor']()
        p.joints = {}
        p.feedback_angles = {}
        p.sliders = {17: namespace['NoWheelSlider'](Qt.Horizontal)}
        p.sliders[17].setRange(-180, 180)
        p.spinboxes = {17: namespace['NoWheelAngleSpinBox']()}
        p.update_3d_robot = Mock()
        p.frame_apply_timer = SimpleNamespace(isActive=lambda: False)
        p.torque_btns = {17: SimpleNamespace(isChecked=lambda: True)}
        p.time_based_profile_ids = set()
        p.online_joints = {17}
        p.port_opened = True
        p.write_goal_positions = Mock(return_value=True)
        p.editing_loaded_pose = False
        p.sliders[17].valueChanged.connect(
            lambda value: p.sync_values(17, value, 'slider'))
        p.spinboxes[17].valueChanged.connect(
            lambda value: p.sync_values(17, value, 'spinbox', transmit_motor=False))
        p.spinboxes[17].lineEdit().returnPressed.connect(
            lambda: p.sync_values(17, p.spinboxes[17].value(), 'spinbox'))
        p.frames = []
        p.motion_sequence = []
        p.saved_sequences = []
        p.max_seq_ms = 500
        p.loaded_sequence_id = None
        p.default_pose_frame_id = None
        p.SCALE = 2.5
        p.time_spinbox = SimpleNamespace(value=lambda: 500)
        p.frame_list_ui1 = Mock()
        p.refresh_library_lists = Mock()

    def test_all_encoder_positions_display_and_round_trip_exactly(self):
        p = self.editor
        for position in range(4096):
            angle = p.dxl_position_to_angle(position)
            p.update_joint_display(17, angle, update_robot=False)
            self.assertEqual(p.spinboxes[17].value(), angle)
            self.assertEqual(p.spinboxes[17].text(), str(int(round(angle))))
            p.spinboxes[17].interpretText()
            self.assertEqual(p.spinboxes[17].value(), angle)
            self.assertEqual(p.joints[17], angle)
            self.assertEqual(p.angle_to_dxl_position(p.joints[17]), position)
        self.assertFalse(p.editing_loaded_pose)
        p.write_goal_positions.assert_not_called()

    def test_live_hand_teaching_keeps_fractional_feedback_and_editor_value(self):
        p = self.editor
        p.online_joints = {17}
        p.is_playing = False
        p.live_angle_read_failures = 0
        namespace['COMM_SUCCESS'] = 0
        p.groupSyncRead = Mock()
        p.groupSyncRead.txRxPacket.return_value = 0
        p.groupSyncRead.isAvailable.return_value = True
        p.groupSyncRead.getData.return_value = 1907
        p.refresh_live_joint_angles()
        self.assertEqual(p.joints[17], -12.392578125)
        self.assertEqual(p.spinboxes[17].value(), -12.392578125)
        self.assertEqual(p.feedback_angles[17], -12.392578125)
        p.write_goal_positions.assert_not_called()

    def test_manual_decimal_input_and_enter_do_not_round_to_slider(self):
        p = self.editor
        p.spinboxes[17].setValue(-12.392578125)
        self.assertEqual(p.joints[17], -12.392578125)
        self.assertEqual(p.sliders[17].value(), -12)
        p.write_goal_positions.assert_not_called()
        QTest.keyClick(p.spinboxes[17], Qt.Key_Return)
        p.write_goal_positions.assert_called_once_with({17: -12.392578125}, target_ids=[17])

    def test_integer_display_preserves_fraction_on_click_enter_focus_loss_and_save(self):
        p = self.editor
        spinbox = p.spinboxes[17]
        p.update_joint_display(17, -32.607421875)
        window = QWidget()
        layout = QVBoxLayout(window)
        layout.addWidget(spinbox)
        other = QLineEdit()
        layout.addWidget(other)
        window.show()
        self.addCleanup(window.close)
        self.app.processEvents()
        QTest.mouseClick(spinbox.lineEdit(), Qt.LeftButton)
        QTest.keyClick(spinbox, Qt.Key_Return)
        QTest.keyClick(spinbox, Qt.Key_Return)
        other.setFocus()
        self.app.processEvents()
        self.assertEqual(spinbox.text(), '-33')
        self.assertEqual(spinbox.value(), -32.607421875)
        self.assertEqual(p.joints[17], -32.607421875)
        self.assertEqual(p.write_goal_positions.call_count, 2)
        p.write_goal_positions.assert_called_with({17: -32.607421875}, target_ids=[17])
        p.add_frame()
        exported = json.loads(json.dumps(p.frames_for_json_export(p.frames)))
        self.assertEqual(exported[0]['angles']['17'], -32.607421875)

    def test_explicitly_typing_the_displayed_integer_replaces_the_fraction(self):
        p = self.editor
        spinbox = p.spinboxes[17]
        p.update_joint_display(17, -32.607421875)
        spinbox.selectAll()
        QTest.keyClicks(spinbox, '-33')
        QTest.keyClick(spinbox, Qt.Key_Return)
        self.assertEqual(spinbox.value(), -33.0)
        self.assertEqual(p.joints[17], -33.0)
        p.write_goal_positions.assert_called_once_with({17: -33.0}, target_ids=[17])

    def test_typed_decimal_displays_as_integer_after_enter_but_keeps_value(self):
        p = self.editor
        spinbox = p.spinboxes[17]
        spinbox.selectAll()
        QTest.keyClicks(spinbox, '-4.746093750')
        QTest.keyClick(spinbox, Qt.Key_Return)
        self.assertEqual(spinbox.text(), '-5')
        self.assertEqual(spinbox.value(), -4.74609375)
        self.assertEqual(p.joints[17], -4.74609375)
        p.write_goal_positions.assert_called_once_with({17: -4.74609375}, target_ids=[17])

    def test_explicit_slider_edit_updates_the_decimal_input(self):
        p = self.editor
        p.update_joint_display(17, -12.392578125)
        p.sliders[17].setValue(-11)
        self.assertEqual(p.joints[17], -11.0)
        self.assertEqual(p.spinboxes[17].value(), -11.0)
        p.write_goal_positions.assert_called_once_with({17: -11.0}, target_ids=[17])

    def test_mouse_arrow_steps_one_degree_and_keeps_fraction_until_enter(self):
        p = self.editor
        spinbox = p.spinboxes[17]
        p.update_joint_display(17, -32.607421875)
        spinbox.resize(90, 40)
        spinbox.show()
        self.addCleanup(spinbox.close)
        self.app.processEvents()
        option = QStyleOptionSpinBox()
        spinbox.initStyleOption(option)
        for control, expected in ((QStyle.SC_SpinBoxUp, -31.607421875),
                                  (QStyle.SC_SpinBoxDown, -32.607421875)):
            rect = spinbox.style().subControlRect(QStyle.CC_SpinBox, option, control, spinbox)
            QTest.mouseClick(spinbox, Qt.LeftButton, pos=rect.center())
            self.assertEqual(spinbox.value(), expected)
            self.assertEqual(spinbox.text(), str(int(round(expected))))
            self.assertEqual(p.joints[17], expected)
        p.write_goal_positions.assert_not_called()
        QTest.keyClick(spinbox, Qt.Key_Return)
        p.write_goal_positions.assert_called_once_with({17: -32.607421875}, target_ids=[17])

    def test_offline_controls_allow_arrow_editing_without_motor_commands(self):
        p = self.editor
        p.online_joints = set()
        button = QPushButton()
        button.setCheckable(True)
        button.setChecked(True)
        p.torque_btns[17] = button
        p.spinboxes[17].setEnabled(False)
        p.sliders[17].setEnabled(False)
        p.set_joint_connection_ui(17, is_online=False)
        self.assertTrue(p.spinboxes[17].isEnabled())
        self.assertTrue(p.sliders[17].isEnabled())
        self.assertFalse(button.isEnabled())
        self.assertFalse(button.isChecked())
        p.update_joint_display(17, 21.796875)
        QTest.keyClick(p.spinboxes[17], Qt.Key_Up)
        self.assertEqual(p.joints[17], 22.796875)
        self.assertEqual(p.spinboxes[17].text(), '23')
        QTest.keyClick(p.spinboxes[17], Qt.Key_Down)
        QTest.keyClick(p.spinboxes[17], Qt.Key_Return)
        self.assertEqual(p.joints[17], 21.796875)
        p.write_goal_positions.assert_not_called()

    def test_stale_torque_on_does_not_send_to_offline_joint(self):
        p = self.editor
        p.online_joints = set()
        p.update_joint_display(17, 21.796875)
        QTest.keyClick(p.spinboxes[17], Qt.Key_Return)
        self.assertEqual(p.joints[17], 21.796875)
        p.write_goal_positions.assert_not_called()

    def teach_frame(self):
        p = self.editor
        p.update_joint_display(17, -12.392578125)
        p.add_frame()
        p.motion_sequence = [dict(copy.deepcopy(p.frames[0]), start_ms=0)]
        p.saved_sequences = [{
            'name': 'taught pose', 'export_enabled': True, 'max_seq_ms': 500,
            'frames': copy.deepcopy(p.motion_sequence),
        }]
        return p.motion_sequence[0]

    def test_teach_save_reload_export_and_gui_playback_use_same_goal(self):
        p = self.editor
        self.teach_frame()
        state = json.loads(json.dumps(p.current_persistent_state_data()))
        collections = (state['frames'], state['motion_sequence'], state['saved_sequences'][0]['frames'])
        for frames in collections:
            reloaded = p.normalize_frame_data(frames[0])
            exported = json.loads(json.dumps(p.frames_for_json_export([reloaded])))[0]
            self.assertEqual(reloaded['angles'][17], -12.392578125)
            self.assertEqual(exported['angles']['17'], -12.392578125)
        player = playback.player()
        player.online_joints = {17}
        player.start_angles = {17: 0.0}
        playback.set_frames(player, state['motion_sequence'])
        player.scrub_timeline(500, force_robot=True)
        self.assertEqual(player.calls[-1], ('goal', {17: -12.392578125}))

    def test_every_json_export_preserves_original_angles_and_metadata(self):
        p = self.editor
        original = self.teach_frame()
        p.spin_motion_repeat = p.spin_composer_repeat = SimpleNamespace(value=lambda: 1)
        p.spin_motion_speed = p.spin_composer_speed = SimpleNamespace(value=lambda: 1.0)
        p.build_composed_sequence_frames = lambda **kwargs: (p.motion_sequence, 500)
        expected_frames = json.loads(json.dumps([original]))
        with TemporaryDirectory() as directory:
            for method in ('export_motion_json', 'export_all_motions_json',
                           'export_checked_motions_json', 'export_composed_sequence_json'):
                with self.subTest(method=method):
                    path = Path(directory) / (method + '.json')
                    namespace['QFileDialog'] = SimpleNamespace(
                        getSaveFileName=lambda *args: (str(path), 'JSON Files (*.json)'))
                    getattr(p, method)()
                    data = json.loads(path.read_text())
                    motion = data['motions'][0] if 'motions' in data else data
                    self.assertEqual(motion['frames'], expected_frames)
        self.assertEqual(original['angles'][17], -12.392578125)

    def test_export_is_an_independent_copy_without_precision_loss(self):
        p = self.editor
        self.teach_frame()
        # A captured interpolated pose can have more digits than the numeric display.
        p.motion_sequence[0]['angles'][17] = -12.39257812512345
        exported = p.frames_for_json_export(p.motion_sequence)
        self.assertEqual(exported, p.motion_sequence)
        exported[0]['angles'][17] = 0.0
        self.assertEqual(p.motion_sequence[0]['angles'][17], -12.39257812512345)


if __name__ == '__main__':
    unittest.main()
