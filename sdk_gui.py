import sys
import math
import fractions
import time
import copy
import json
import os
import signal
import struct
import uuid
import xml.etree.ElementTree as ET
# 아무거나나ㅏㅏㅏ

# 🚨 [필수] 파이썬 3.9+ fractions.gcd -> math.gcd 긴급 호환성 패치
fractions.gcd = math.gcd

# 🚨 [필수] 파이썬 3.10+ collections 호환성 완벽 끝판왕 패치
import collections
import collections.abc
for name in dir(collections.abc):
    if not name.startswith('_'):
        setattr(collections, name, getattr(collections.abc, name))

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QSlider, QLabel, QPushButton, QListWidget, QFileDialog, 
                             QSpinBox, QDoubleSpinBox, QGroupBox, QScrollArea, QFrame, QInputDialog,
                             QTabWidget, QMessageBox, QGridLayout, QCheckBox, QListWidgetItem, QAbstractItemView,
                             QOpenGLWidget)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen, QMatrix4x4

# 🔥 다이나믹셀 SDK 라이브러리 임포트
from dynamixel_sdk import *
from dynamixel_sdk.protocol2_packet_handler import Protocol2PacketHandler

# 🤖 하드웨어 및 다이나믹셀 통신 주소 세팅 (MX 시리즈 프로토콜 2.0 기준)
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
LEN_GOAL_POSITION           = 4         
LEN_PRESENT_POSITION        = 4         
PROTOCOL_VERSION            = 2.0       
BAUDRATE                    = 4000000   
DEVICENAME                  = '/dev/ttyUSB0' # (자동 탐색기가 완전히 실패했을 때의 최후의 백업 포트)

# DYNAMIXEL absolute position 기준: 0~4095, 2048이 중립, 4096 step/rev.
DXL_POSITION_RESOLUTION     = 4096
DXL_CENTER_POSITION         = 2048
DXL_MIN_POSITION            = 0
DXL_MAX_POSITION            = 4095
DXL_DEG_PER_STEP            = 360.0 / DXL_POSITION_RESOLUTION
DXL_STEPS_PER_DEG           = DXL_POSITION_RESOLUTION / 360.0
DXL_MIN_DEG                 = (DXL_MIN_POSITION - DXL_CENTER_POSITION) * DXL_DEG_PER_STEP
DXL_MAX_DEG                 = (DXL_MAX_POSITION - DXL_CENTER_POSITION) * DXL_DEG_PER_STEP

# 0이면 STL을 줄이지 않고 원본 mesh를 렌더링합니다. 중간 샘플링은 표면이 찢어져 보여서 기본은 원본 유지.
MAX_RENDER_TRIANGLES_PER_MESH = 0
STATE_FILENAME = "sdk_gui_state.json"
MIN_TIMELINE_FRAME_MS = 10


class SafeProtocol2PacketHandler(Protocol2PacketHandler):
    def _release_port(self, port):
        if hasattr(port, "is_using"):
            port.is_using = False

    def txPacket(self, port, txpacket):
        if getattr(port, "is_using", False):
            self._release_port(port)
        try:
            return super().txPacket(port, txpacket)
        except Exception as exc:
            self._release_port(port)
            print(f"[❌ TX 예외] 다이나믹셀 송신 중 예외 발생: {exc}")
            return COMM_TX_FAIL
        finally:
            self._release_port(port)

    def rxPacket(self, port, fast_option):
        try:
            return super().rxPacket(port, fast_option)
        except Exception as exc:
            self._release_port(port)
            print(f"[❌ RX 예외] 다이나믹셀 수신 중 예외 발생: {exc}")
            return [], COMM_RX_FAIL
        finally:
            self._release_port(port)

    def txRxPacket(self, port, txpacket):
        try:
            return super().txRxPacket(port, txpacket)
        except Exception as exc:
            self._release_port(port)
            print(f"[❌ TX/RX 예외] 다이나믹셀 통신 중 예외 발생: {exc}")
            return None, COMM_TX_FAIL, 0
        finally:
            self._release_port(port)

    def read1ByteTxRx(self, port, dxl_id, address):
        try:
            return super().read1ByteTxRx(port, dxl_id, address)
        except (IndexError, TypeError):
            # SDK가 성공 코드와 함께 길이가 부족한 상태 패킷을 돌려주면
            # 내부에서 data[0] 접근 중 예외가 발생할 수 있습니다.
            self._release_port(port)
            return 0, COMM_RX_FAIL, 0

    def read4ByteTxRx(self, port, dxl_id, address):
        try:
            return super().read4ByteTxRx(port, dxl_id, address)
        except (IndexError, TypeError):
            # SDK의 COMM_SUCCESS 판정과 무관하게 실제 데이터가 4바이트보다
            # 짧은 경우를 정상적인 통신 실패 반환값으로 변환합니다.
            self._release_port(port)
            return 0, COMM_RX_FAIL, 0


class NoWheelSpinBox(QSpinBox):
    """마우스 휠로 값이 우발적으로 바뀌지 않는 숫자 입력칸."""

    def wheelEvent(self, event):
        event.ignore()


class NoWheelSlider(QSlider):
    """마우스 휠은 목록 스크롤에만 사용하고 관절값은 변경하지 않습니다."""

    def wheelEvent(self, event):
        event.ignore()


# ----------------------------------------------------
# 리스트 커스텀 항목 위젯 (1번 탭 라이브러리용)
# ----------------------------------------------------
class FrameItemWidget(QWidget):
    def __init__(self, frame_data, parent_gui):
        super().__init__()
        self.frame_data = frame_data
        self.parent_gui = parent_gui
        
        layout = QHBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.checkbox = QCheckBox()
        self.checkbox.setVisible(self.parent_gui.is_select_mode)
        self.checkbox.setStyleSheet("spacing: 10px;")
        
        self.label = QLabel(f"[{frame_data['name']}] {frame_data['time_ms']}ms")
        self.label.setStyleSheet("font-size: 13pt;")
        
        self.btn_star = QPushButton('★' if frame_data.get('is_important') else '☆')
        self.btn_star.setFlat(True)
        self.btn_star.setFixedWidth(50)
        self.btn_star.setStyleSheet("font-size: 20pt; color: #FFD700; border: none; background: transparent;")
        self.btn_star.clicked.connect(self.toggle_star)
        
        layout.addWidget(self.checkbox)
        layout.addWidget(self.label)
        layout.addStretch()
        layout.addWidget(self.btn_star)
        
        self.setLayout(layout)
        
    def toggle_star(self):
        is_imp = not self.frame_data.get('is_important', False)
        self.frame_data['is_important'] = is_imp
        self.btn_star.setText('★' if is_imp else '☆')
        self.parent_gui.refresh_library_lists()

# ----------------------------------------------------
# 타임라인 클립 위젯
# ----------------------------------------------------
class TimelineBlockWidget(QFrame):
    def __init__(self, frame_data, seq_idx, parent_gui):
        super().__init__()
        self.frame_data = frame_data
        self.parent_gui = parent_gui
        self.seq_idx = seq_idx

        self.setFrameShape(QFrame.StyledPanel)
        self.set_default_style()
        # 블록의 실제 픽셀 폭이 곧 시간 폭입니다. 고정 60px 최소 폭은 짧은
        # 프레임을 약 160~240ms처럼 보이게 하고 이웃 블록 위에 덮어씌웠습니다.
        self.setMinimumWidth(1)
        
        self.setMouseTracking(True) 
        self.EDGE_MARGIN = 15 
        self.drag_mode = None 

        layout = QVBoxLayout()
        layout.setContentsMargins(2, 2, 2, 2)
        
        top_layout = QHBoxLayout()
        lbl_title = QLabel(f"[{seq_idx+1}] {frame_data['name']}")
        lbl_title.setStyleSheet("font-weight: bold; color: white; border: none; background: transparent; font-size: 11pt;")
        
        btn_del = QPushButton("❌")
        btn_del.setFixedWidth(20)
        btn_del.setStyleSheet("background-color: transparent; border: none; color: #ff5252; font-weight: bold; font-size: 10pt;")
        btn_del.clicked.connect(self.on_delete)
        
        top_layout.addWidget(lbl_title)
        top_layout.addWidget(btn_del)
        
        self.spinbox = QSpinBox()
        self.spinbox.setRange(10, self.parent_gui.max_seq_ms)
        self.spinbox.setValue(frame_data['time_ms'])
        self.spinbox.setSuffix(" ms")
        self.spinbox.setStyleSheet("font-size: 10pt; border: 1px solid #777; background: #222; color: white; padding: 2px;")
        # 숫자를 입력하는 도중에는 블록 길이를 바꾸지 않고 Enter로 확정합니다.
        self.spinbox.setKeyboardTracking(False)
        self.spinbox.lineEdit().returnPressed.connect(self.apply_spinbox_time)

        self.gap_spinbox = QSpinBox()
        self.gap_spinbox.setRange(0, self.parent_gui.max_seq_ms)
        self.gap_spinbox.setValue(self.parent_gui.frame_gap_ms(frame_data))
        self.gap_spinbox.setSuffix(" ms 앞 프레임 간격")
        self.gap_spinbox.setKeyboardTracking(False)
        self.gap_spinbox.setStyleSheet(
            "font-size: 9pt; border: 1px solid #777; background: #222; color: #ffd54f; padding: 1px;"
        )
        self.gap_spinbox.lineEdit().returnPressed.connect(self.apply_gap_time)
        
        layout.addLayout(top_layout)
        layout.addWidget(self.gap_spinbox)
        layout.addWidget(self.spinbox)
        layout.addStretch() 
        
        self.setLayout(layout)

    def set_default_style(self):
        self.setStyleSheet("TimelineBlockWidget { background-color: #444; border: 2px solid #555; border-radius: 6px; }")

    def set_playing_style(self):
        self.setStyleSheet("TimelineBlockWidget { background-color: #2e7d32; border: 3px solid #69f0ae; border-radius: 6px; }")

    def calculate_bounds(self):
        seq = sorted(self.parent_gui.motion_sequence, key=lambda x: x['start_ms'])
        idx = next((i for i, frame in enumerate(seq) if frame is self.frame_data), -1)

        self.resize_min_x = 0
        if idx > 0:
            prev_f = seq[idx-1]
            self.resize_min_x = int((prev_f['start_ms'] + prev_f['time_ms']) * self.parent_gui.SCALE)

        self.resize_max_x = int(self.parent_gui.max_seq_ms * self.parent_gui.SCALE)
        if idx != -1 and idx < len(seq) - 1:
            next_f = seq[idx+1]
            self.resize_max_x = int(next_f['start_ms'] * self.parent_gui.SCALE)

        self.move_min_x = self.resize_min_x
        self.move_max_x = self.resize_max_x - self.width()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            edge_margin = min(self.EDGE_MARGIN, max(1, self.width() // 3))
            if event.x() < edge_margin: self.drag_mode = 'resize_left'
            elif event.x() >= self.width() - edge_margin: self.drag_mode = 'resize_right'
            else: self.drag_mode = 'move'

            if self.drag_mode == 'move':
                self.frame_data['_drag_original_start_ms'] = self.frame_data['start_ms']
                
            self.drag_start_global_x = event.globalX()
            self.start_x = self.x()
            self.start_w = self.width()
            self.calculate_bounds()
            self.raise_() 

    def mouseMoveEvent(self, event):
        if not self.drag_mode:
            edge_margin = min(self.EDGE_MARGIN, max(1, self.width() // 3))
            if event.x() < edge_margin or event.x() >= self.width() - edge_margin:
                self.setCursor(Qt.SizeHorCursor) 
            else:
                self.setCursor(Qt.ArrowCursor)   
            return

        delta_x = event.globalX() - self.drag_start_global_x
        
        if self.drag_mode == 'move':
            new_x = self.start_x + delta_x
            max_x = int(self.parent_gui.max_seq_ms * self.parent_gui.SCALE) - self.width()
            new_x = max(0, min(new_x, max_x))
            self.move(new_x, self.y())
            self.frame_data['start_ms'] = int(new_x / self.parent_gui.SCALE)
            
        elif self.drag_mode == 'resize_right':
            new_w = self.start_w + delta_x
            max_w = self.resize_max_x - self.start_x
            min_w = max(1, int(math.ceil(MIN_TIMELINE_FRAME_MS * self.parent_gui.SCALE)))
            new_w = max(min_w, min(new_w, max_w))
            self.resize(new_w, self.height())
            
            new_time_ms = int(new_w / self.parent_gui.SCALE)
            self.frame_data['time_ms'] = new_time_ms
            self.spinbox.blockSignals(True)
            self.spinbox.setValue(new_time_ms)
            self.spinbox.blockSignals(False)
            
        elif self.drag_mode == 'resize_left':
            new_x = self.start_x + delta_x
            new_w = self.start_w - delta_x
            
            if new_x < self.resize_min_x:
                diff = self.resize_min_x - new_x
                new_x += diff
                new_w -= diff
            
            min_w = max(1, int(math.ceil(MIN_TIMELINE_FRAME_MS * self.parent_gui.SCALE)))
            if new_w < min_w:
                diff = min_w - new_w
                new_w += diff
                new_x -= diff
                
            self.move(new_x, self.y())
            self.resize(new_w, self.height())
            
            self.frame_data['start_ms'] = int(new_x / self.parent_gui.SCALE)
            self.frame_data['time_ms'] = int(new_w / self.parent_gui.SCALE)
            self.spinbox.blockSignals(True)
            self.spinbox.setValue(self.frame_data['time_ms'])
            self.spinbox.blockSignals(False)

        self.parent_gui.refresh_timeline_meta() 

    def mouseReleaseEvent(self, event):
        if self.drag_mode:
            released_mode = self.drag_mode
            self.drag_mode = None
            self.setCursor(Qt.ArrowCursor)
            if released_mode == 'move':
                self.parent_gui.reorder_motion_frame(self.frame_data, self.x())
            else:
                self.parent_gui.resort_motion_sequence()
            self.parent_gui.refresh_timeline_ui()

    def clamp_time_val(self, val):
        self.calculate_bounds()
        max_allowed_ms = int((self.resize_max_x - self.x()) / self.parent_gui.SCALE)
        return min(val, max_allowed_ms)

    def apply_spinbox_time(self):
        self.spinbox.interpretText()
        val = self.spinbox.value()
        val = self.clamp_time_val(val) 
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(val)
        self.spinbox.blockSignals(False)
        self.frame_data['time_ms'] = val
        self.parent_gui.refresh_timeline_ui()

    def apply_gap_time(self):
        self.gap_spinbox.interpretText()
        requested_gap = self.gap_spinbox.value()
        previous_end = self.parent_gui.previous_frame_end_ms(self.frame_data)
        requested_start = previous_end + requested_gap
        self.frame_data['_drag_original_start_ms'] = self.frame_data['start_ms']
        self.parent_gui.reorder_motion_frame(
            self.frame_data,
            int(requested_start * self.parent_gui.SCALE),
        )
        self.parent_gui.refresh_timeline_ui()

    def on_delete(self):
        self.parent_gui.remove_from_motion_by_idx(self.seq_idx)

# ----------------------------------------------------
# 캔버스 (타임라인 눈금자 및 배경 + 드래그 앤 드롭 수신)
# ----------------------------------------------------
class TimelineContainer(QWidget):
    def __init__(self, parent_gui):
        super().__init__()
        self.parent_gui = parent_gui
        self.show_playhead = False
        self.playhead_x = 0
        self.setAcceptDrops(True) 

    def set_playhead(self, show, x=0):
        self.show_playhead = show
        self.playhead_x = x
        self.update() 

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        
        pen_axis = QPen(QColor(120, 120, 120))
        pen_axis.setWidth(2)
        painter.setPen(pen_axis)
        y_axis = 30 
        
        max_w = int(self.parent_gui.max_seq_ms * self.parent_gui.SCALE)
        painter.drawLine(0, y_axis, max_w, y_axis) 
        
        # 확대 상태에서는 10ms 단위 눈금을 표시해 짧은 프레임도 정밀 배치합니다.
        tick_interval_ms = 10 if self.parent_gui.SCALE >= 0.75 else 50
        label_interval_ms = 100 if self.parent_gui.SCALE >= 0.75 else 500
        current_ms = 0
        
        while current_ms <= self.parent_gui.max_seq_ms:
            x = int(current_ms * self.parent_gui.SCALE)
            if current_ms % label_interval_ms == 0:
                painter.drawLine(x, y_axis - 10, x, y_axis)
                painter.setPen(QColor(200, 200, 200))
                painter.drawText(x + 3, y_axis - 5, f"{current_ms}ms")
                painter.setPen(pen_axis)
            elif current_ms % 100 == 0:
                painter.drawLine(x, y_axis - 6, x, y_axis)
            else:
                painter.drawLine(x, y_axis - 3, x, y_axis)
            current_ms += tick_interval_ms

        if self.show_playhead:
            pen_playhead = QPen(QColor(255, 152, 0)) 
            pen_playhead.setWidth(3)
            painter.setPen(pen_playhead)
            painter.drawLine(self.playhead_x, 0, self.playhead_x, self.height())

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.parent_gui.is_playing:
                self.parent_gui.pause_motion_sequence()
            self.scrub(event.x())

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            self.scrub(event.x())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            # 마지막 목표 자세도 완만한 로봇 추종기에 전달합니다.
            self.scrub(event.x())

    def scrub(self, x, force_robot=False):
        max_w = int(self.parent_gui.max_seq_ms * self.parent_gui.SCALE)
        x = max(0, min(x, max_w))
        self.set_playhead(True, x)
        t_ms = int(x / self.parent_gui.SCALE)
        self.parent_gui.scrub_timeline(t_ms, force_robot=force_robot)

    def dragEnterEvent(self, event):
        event.accept()


class SequenceTimelineBlockWidget(QFrame):
    """3페이지에서 저장 시퀀스 하나를 나타내는 타임라인 블록."""

    def __init__(self, entry, index, parent_gui):
        super().__init__()
        self.entry = entry
        self.index = index
        self.parent_gui = parent_gui
        self.drag_start_global_x = 0
        self.start_x = 0

        self.setStyleSheet(
            "SequenceTimelineBlockWidget { background-color: #5e35b1; "
            "border: 2px solid #9575cd; border-radius: 6px; }"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        top = QHBoxLayout()
        label = QLabel(f"[{index + 1}] {entry['name']}")
        label.setStyleSheet("color: white; font-weight: bold; font-size: 11pt;")
        delete_button = QPushButton("❌")
        delete_button.setFixedWidth(24)
        delete_button.setStyleSheet("background: transparent; border: none; color: #ff8a80;")
        delete_button.clicked.connect(lambda: self.parent_gui.remove_sequence_composer_entry(self.index))
        top.addWidget(label)
        top.addStretch()
        top.addWidget(delete_button)
        duration = QLabel(f"{entry['time_ms']}ms / {entry['frame_count']} 프레임")
        duration.setStyleSheet("color: white;")
        layout.addLayout(top)
        layout.addWidget(duration)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_start_global_x = event.globalX()
            self.start_x = self.x()
            self.raise_()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            new_x = self.start_x + event.globalX() - self.drag_start_global_x
            max_x = max(0, self.parent_gui.composer_timeline_width() - self.width())
            self.move(max(0, min(new_x, max_x)), self.y())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.parent_gui.reorder_sequence_composer_entry(self.entry, self.x())


class SequenceTimelineContainer(QWidget):
    def __init__(self, parent_gui):
        super().__init__()
        self.parent_gui = parent_gui
        self.show_playhead = False
        self.playhead_x = 0
        self.setAcceptDrops(True)

    def set_playhead(self, show, x=0):
        self.show_playhead = show
        self.playhead_x = x
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        axis_pen = QPen(QColor(120, 120, 120))
        axis_pen.setWidth(2)
        painter.setPen(axis_pen)
        painter.drawLine(0, 30, self.parent_gui.composer_timeline_width(), 30)
        duration = self.parent_gui.composer_total_duration()
        tick = 100
        current = 0
        while current <= duration:
            x = int(current * self.parent_gui.COMPOSER_SCALE)
            if current % 500 == 0:
                painter.drawLine(x, 20, x, 30)
                painter.drawText(x + 3, 16, f"{current}ms")
            else:
                painter.drawLine(x, 26, x, 30)
            current += tick
        if self.show_playhead:
            playhead_pen = QPen(QColor(255, 152, 0))
            playhead_pen.setWidth(3)
            painter.setPen(playhead_pen)
            painter.drawLine(self.playhead_x, 0, self.playhead_x, self.height())

    def dragEnterEvent(self, event):
        event.accept()

    def dragMoveEvent(self, event):
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.parent_gui.is_playing:
                self.parent_gui.pause_motion_sequence()
            self.scrub(event.x())

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            self.scrub(event.x())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.scrub(event.x())

    def scrub(self, x):
        if not self.parent_gui.sequence_composer_entries:
            return
        if self.parent_gui.playback_context != "composer":
            if not self.parent_gui.activate_composer_playback():
                return
        max_x = int(self.parent_gui.composer_total_duration() * self.parent_gui.COMPOSER_SCALE)
        x = max(0, min(x, max_x))
        self.set_playhead(True, x)
        self.parent_gui.scrub_timeline(int(x / self.parent_gui.COMPOSER_SCALE))

    def dropEvent(self, event):
        item = self.parent_gui.composer_source_list.currentItem()
        if item is None:
            return
        self.parent_gui.add_sequence_to_composer(
            sequence_idx=item.data(Qt.UserRole),
            drop_x=event.pos().x(),
        )
        event.accept()

    def dragMoveEvent(self, event):
        event.accept()

    def dropEvent(self, event):
        selected_items = self.parent_gui.frame_list_all.selectedItems()
        if not selected_items: return

        drop_x = event.pos().x()
        start_ms = int(drop_x / self.parent_gui.SCALE)

        for item in selected_items:
            original_idx = item.data(Qt.UserRole)
            new_frame = copy.deepcopy(self.parent_gui.frames[original_idx])
            new_frame['time_ms'] = max(MIN_TIMELINE_FRAME_MS, new_frame['time_ms'])
            
            new_frame['start_ms'] = start_ms
            self.parent_gui.motion_sequence.append(new_frame)
            placed = self.parent_gui.reorder_motion_frame(
                new_frame, int(start_ms * self.parent_gui.SCALE)
            )
            if not placed:
                QMessageBox.warning(self, "공간 부족", "겹치지 않고 배치할 빈 공간이 없습니다.")
                break
            start_ms = new_frame['start_ms'] + new_frame['time_ms']

        self.parent_gui.refresh_timeline_ui()
        self.parent_gui.frame_list_all.clearSelection()
        event.accept()


# ----------------------------------------------------
# 내장 URDF/STL OpenGL 뷰어 (urdfpy/pyqtgraph 없이 동작)
# ----------------------------------------------------
def _parse_vec3(text, default=(0.0, 0.0, 0.0)):
    if not text:
        return default
    vals = [float(v) for v in text.split()]
    if len(vals) != 3:
        return default
    return tuple(vals)


def _transform_from_xyz_rpy(xyz, rpy):
    import numpy as np
    x, y, z = xyz
    roll, pitch, yaw = rpy

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)

    mat = np.eye(4, dtype=float)
    mat[:3, :3] = rz @ ry @ rx
    mat[:3, 3] = [x, y, z]
    return mat


def _axis_angle_transform(axis, angle):
    import numpy as np
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        return np.eye(4, dtype=float)

    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c

    mat = np.eye(4, dtype=float)
    mat[:3, :3] = np.array([
        [t*x*x + c,     t*x*y - s*z,   t*x*z + s*y],
        [t*x*y + s*z,   t*y*y + c,     t*y*z - s*x],
        [t*x*z - s*y,   t*y*z + s*x,   t*z*z + c],
    ], dtype=float)
    return mat


def _load_stl_triangles(path, max_triangles=MAX_RENDER_TRIANGLES_PER_MESH):
    triangles = []
    with open(path, "rb") as f:
        data = f.read()

    is_binary = False
    if len(data) >= 84:
        tri_count = struct.unpack_from("<I", data, 80)[0]
        is_binary = 84 + tri_count * 50 == len(data)

    if is_binary:
        stride = max(1, math.ceil(tri_count / max_triangles)) if max_triangles else 1
        for tri_idx in range(0, tri_count, stride):
            if max_triangles and len(triangles) >= max_triangles:
                break
            offset = 84 + tri_idx * 50
            vals = struct.unpack_from("<12fH", data, offset)
            normal = vals[0:3]
            v1 = vals[3:6]
            v2 = vals[6:9]
            v3 = vals[9:12]
            triangles.append((normal, v1, v2, v3))
        return triangles

    text = data.decode("utf-8", errors="ignore").splitlines()
    normal = (0.0, 0.0, 1.0)
    vertices = []
    for line in text:
        parts = line.strip().split()
        if not parts:
            continue
        if parts[0] == "facet" and len(parts) >= 5 and parts[1] == "normal":
            normal = (float(parts[2]), float(parts[3]), float(parts[4]))
        elif parts[0] == "vertex" and len(parts) >= 4:
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            if len(vertices) == 3:
                triangles.append((normal, vertices[0], vertices[1], vertices[2]))
                vertices = []
    if max_triangles and len(triangles) > max_triangles:
        stride = max(1, math.ceil(len(triangles) / max_triangles))
        triangles = triangles[::stride][:max_triangles]
    return triangles


class SimpleURDFModel:
    def __init__(self):
        self.links = set()
        self.root_link = None
        self.joints = {}
        self.children = {}
        self.visuals = []
        self.mesh_cache = {}
        self.triangle_count = 0
        self.bounds_min = None
        self.bounds_max = None

    @classmethod
    def load(cls, urdf_path):
        import numpy as np

        model = cls()
        base_dir = os.path.dirname(os.path.abspath(urdf_path))
        root = ET.parse(urdf_path).getroot()

        for link in root.findall("link"):
            link_name = link.get("name")
            model.links.add(link_name)
            for visual in link.findall("visual"):
                origin = visual.find("origin")
                xyz = _parse_vec3(origin.get("xyz") if origin is not None else None)
                rpy = _parse_vec3(origin.get("rpy") if origin is not None else None)

                mesh = visual.find("./geometry/mesh")
                if mesh is None:
                    continue
                mesh_filename = mesh.get("filename", "")
                mesh_filename = mesh_filename.replace("package://step/", "")
                mesh_path = mesh_filename if os.path.isabs(mesh_filename) else os.path.join(base_dir, mesh_filename)
                if not os.path.exists(mesh_path):
                    print(f"[URDF 경고] Mesh 파일 없음: {mesh_path}")
                    continue

                material_color = (0.85, 0.88, 0.92, 1.0)
                color = visual.find("./material/color")
                if color is not None and color.get("rgba"):
                    vals = [float(v) for v in color.get("rgba").split()]
                    if len(vals) == 4:
                        material_color = (vals[0], vals[1], vals[2], 1.0)

                if mesh_path not in model.mesh_cache:
                    model.mesh_cache[mesh_path] = _load_stl_triangles(mesh_path)

                visual_entry = {
                    "link": link_name,
                    "origin": _transform_from_xyz_rpy(xyz, rpy),
                    "mesh": mesh_path,
                    "color": material_color,
                }
                model.visuals.append(visual_entry)

        child_links = set()
        for joint in root.findall("joint"):
            joint_name = joint.get("name")
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue

            origin = joint.find("origin")
            xyz = _parse_vec3(origin.get("xyz") if origin is not None else None)
            rpy = _parse_vec3(origin.get("rpy") if origin is not None else None)
            axis_node = joint.find("axis")
            axis = _parse_vec3(axis_node.get("xyz") if axis_node is not None else None, (0.0, 0.0, 1.0))

            parent_link = parent.get("link")
            child_link = child.get("link")
            child_links.add(child_link)
            joint_data = {
                "name": joint_name,
                "type": joint.get("type", "fixed"),
                "parent": parent_link,
                "child": child_link,
                "origin": _transform_from_xyz_rpy(xyz, rpy),
                "axis": axis,
            }
            model.joints[joint_name] = joint_data
            model.children.setdefault(parent_link, []).append(joint_data)

        root_candidates = sorted(model.links - child_links)
        model.root_link = root_candidates[0] if root_candidates else "base_link"
        model.triangle_count = sum(len(triangles) for triangles in model.mesh_cache.values())
        model._compute_bounds()
        return model

    def _compute_link_poses(self, joint_angles):
        import numpy as np

        poses = {self.root_link: np.eye(4, dtype=float)}
        stack = [self.root_link]
        while stack:
            parent_link = stack.pop()
            parent_pose = poses[parent_link]
            for joint in self.children.get(parent_link, []):
                angle = joint_angles.get(joint["name"], 0.0)
                if joint["type"] in ("revolute", "continuous"):
                    motion = _axis_angle_transform(joint["axis"], angle)
                else:
                    motion = np.eye(4, dtype=float)
                poses[joint["child"]] = parent_pose @ joint["origin"] @ motion
                stack.append(joint["child"])
        return poses

    def _compute_bounds(self):
        import numpy as np

        poses = self._compute_link_poses({})
        mins = []
        maxs = []
        for visual in self.visuals:
            pose = poses.get(visual["link"], np.eye(4, dtype=float)) @ visual["origin"]
            triangles = self.mesh_cache.get(visual["mesh"], [])
            if not triangles:
                continue
            pts = np.array([vertex for _, v1, v2, v3 in triangles for vertex in (v1, v2, v3)], dtype=float)
            pts_h = np.c_[pts, np.ones(len(pts))]
            world = (pose @ pts_h.T).T[:, :3]
            mins.append(world.min(axis=0))
            maxs.append(world.max(axis=0))

        if mins:
            self.bounds_min = np.vstack(mins).min(axis=0)
            self.bounds_max = np.vstack(maxs).max(axis=0)
        else:
            self.bounds_min = np.array([-0.2, -0.2, -0.2], dtype=float)
            self.bounds_max = np.array([0.2, 0.2, 0.2], dtype=float)


class URDFGLViewer(QOpenGLWidget):
    def __init__(self, robot_model, parent=None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.joint_angles = {}
        self.setMinimumWidth(350)
        self.setMinimumHeight(300)
        # URDF는 Z-up인데 OpenGL 카메라는 기본적으로 -Z 방향을 봅니다.
        # X축 회전을 음수로 두어 로봇의 +Z가 화면 위쪽으로 오게 맞춥니다.
        self.rot_x = -65.0
        self.rot_z = 25.0
        self.distance = 2.2
        self.last_mouse_pos = None
        self.mesh_display_lists = {}
        self.pending_joint_angles = {}
        self.update_pending = False

        import numpy as np
        center = (self.robot_model.bounds_min + self.robot_model.bounds_max) / 2.0
        size = self.robot_model.bounds_max - self.robot_model.bounds_min
        max_size = max(float(size.max()), 0.001)
        self.model_center = center
        self.model_scale = 1.45 / max_size

    def set_joint_angles(self, joint_angles):
        self.pending_joint_angles = dict(joint_angles)
        if self.update_pending:
            return
        self.update_pending = True
        QTimer.singleShot(33, self.flush_joint_update)

    def flush_joint_update(self):
        self.joint_angles = self.pending_joint_angles
        self.update_pending = False
        self.update()

    def initializeGL(self):
        from OpenGL.GL import (
            glBegin, glBlendFunc, glClearColor, glColorMaterial, glEnable, glEnd,
            glEndList, glGenLists, glLightfv, glNewList, glNormal3f,
            glShadeModel, GL_AMBIENT, GL_AMBIENT_AND_DIFFUSE, GL_BLEND,
            GL_COLOR_MATERIAL, GL_DEPTH_TEST, GL_DIFFUSE, GL_FRONT_AND_BACK,
            GL_LIGHT0, GL_LIGHTING, GL_NORMALIZE, GL_ONE_MINUS_SRC_ALPHA,
            GL_POSITION, GL_SMOOTH, GL_SRC_ALPHA, GL_COMPILE, GL_TRIANGLES,
            glVertex3f
        )

        glClearColor(0.13, 0.13, 0.13, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_NORMALIZE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glShadeModel(GL_SMOOTH)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, [1.2, -1.6, 2.4, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.35, 0.35, 0.35, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.82, 0.82, 0.82, 1.0])

        self.mesh_display_lists.clear()
        for mesh_path, triangles in self.robot_model.mesh_cache.items():
            list_id = glGenLists(1)
            glNewList(list_id, GL_COMPILE)
            glBegin(GL_TRIANGLES)
            for normal, v1, v2, v3 in triangles:
                glNormal3f(float(normal[0]), float(normal[1]), float(normal[2]))
                glVertex3f(float(v1[0]), float(v1[1]), float(v1[2]))
                glVertex3f(float(v2[0]), float(v2[1]), float(v2[2]))
                glVertex3f(float(v3[0]), float(v3[1]), float(v3[2]))
            glEnd()
            glEndList()
            self.mesh_display_lists[mesh_path] = list_id

    def resizeGL(self, width, height):
        from OpenGL.GL import glMatrixMode, glLoadIdentity, glViewport, GL_MODELVIEW, GL_PROJECTION
        from OpenGL.GLU import gluPerspective

        height = max(1, height)
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, width / float(height), 0.01, 20.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        import numpy as np
        from OpenGL.GL import (
            glCallList, glClear, glColor4f, glLoadIdentity, glMatrixMode,
            glMultMatrixf, glPopMatrix, glPushMatrix, glRotatef, glScalef,
            glTranslatef, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_MODELVIEW
        )

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -self.distance)
        glRotatef(self.rot_x, 1.0, 0.0, 0.0)
        glRotatef(self.rot_z, 0.0, 0.0, 1.0)
        glScalef(self.model_scale, self.model_scale, self.model_scale)
        glTranslatef(-float(self.model_center[0]), -float(self.model_center[1]), -float(self.model_center[2]))

        link_poses = self.robot_model._compute_link_poses(self.joint_angles)
        for visual in self.robot_model.visuals:
            pose = link_poses.get(visual["link"])
            if pose is None:
                continue
            transform = pose @ visual["origin"]
            color = visual["color"]
            list_id = self.mesh_display_lists.get(visual["mesh"])
            if list_id is None:
                continue

            glPushMatrix()
            glMultMatrixf(transform.T.astype(np.float32).flatten())
            glColor4f(float(color[0]), float(color[1]), float(color[2]), float(color[3]))
            glCallList(list_id)
            glPopMatrix()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is None:
            return
        dx = event.x() - self.last_mouse_pos.x()
        dy = event.y() - self.last_mouse_pos.y()
        self.rot_z += dx * 0.5
        self.rot_x += dy * 0.5
        self.last_mouse_pos = event.pos()
        self.update()

    def mouseReleaseEvent(self, event):
        self.last_mouse_pos = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.distance *= 0.9
        else:
            self.distance *= 1.1
        self.distance = max(0.3, min(8.0, self.distance))
        self.update()


# ----------------------------------------------------
# 메인 에디터 클래스
# ----------------------------------------------------
class SDKMotionEditor(QWidget):
    def __init__(self):
        super().__init__()
        # 1ms = 1px가 기본값입니다. 가로 스크롤로 10ms 프레임까지 편집합니다.
        self.SCALE = 1.0
        self.max_seq_ms = 5000 
        
        self.frames = []          
        self.motion_sequence = [] 
        self.saved_sequences = [] 
        self.is_select_mode = False 
        self.autosave_enabled = False
        self.state_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), STATE_FILENAME)
        
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self.anim_step)
        self.live_angle_timer = QTimer(self)
        self.live_angle_timer.setInterval(100)
        self.live_angle_timer.timeout.connect(self.refresh_live_joint_angles)
        self.live_angle_read_failures = 0
        self.robot_scrub_timer = QTimer(self)
        self.robot_scrub_timer.setInterval(30)
        self.robot_scrub_timer.timeout.connect(self.step_robot_scrub)
        self.robot_scrub_target_angles = {}
        self.robot_scrub_command_angles = {}
        self.robot_scrub_last_time = 0.0
        self.robot_scrub_speed_deg_per_sec = 45.0
        self.frame_apply_timer = QTimer(self)
        self.frame_apply_timer.setInterval(30)
        self.frame_apply_timer.timeout.connect(self.step_frame_apply)
        self.frame_apply_start_time = 0.0
        self.frame_apply_duration = 0.5
        self.frame_apply_start_angles = {}
        self.frame_apply_target_angles = {}
        self.frame_apply_ids = []
        self.frame_apply_name = ""
        self.frame_apply_on_complete = None
        self.is_playing = False
        self.is_paused = False
        self.playback_context = "motion"
        self.composer_motion_backup = None
        self.playback_repeat_target = 1
        self.playback_repeat_current = 1
        self.playback_speed = 1.0
        self.current_timeline_ms = 0
        self.current_seq_idx = 0
        self.anim_start_time = 0
        self.anim_duration = 0
        self.start_angles = {}
        
        self.execute_on_real_robot = False
        self.robot_sync_enabled = False
        self.last_robot_write_time = 0
        
        self.joint_data = [
            {"id": 0, "name": "Head_Pan", "type": "28"}, {"id": 1, "name": "Head_Tilt", "type": "28"},
            {"id": 2, "name": "R_Shoulder_Pitch", "type": "64"}, {"id": 3, "name": "L_Shoulder_Pitch", "type": "64"},
            {"id": 4, "name": "R_Shoulder_Roll", "type": "28"}, {"id": 5, "name": "L_Shoulder_Roll", "type": "28"},
            {"id": 6, "name": "R_Elbow_Pitch", "type": "28"}, {"id": 7, "name": "L_Elbow_Pitch", "type": "28"},
            {"id": 8, "name": "R_Wrist_Yaw", "type": "28"}, {"id": 9, "name": "L_Wrist_Yaw", "type": "28"},
            {"id": 10, "name": "R_Hip_Yaw", "type": "106"}, {"id": 11, "name": "Waist_Yaw", "type": "106"},      
            {"id": 12, "name": "L_Hip_Yaw", "type": "106"}, {"id": 15, "name": "R_Hip_Roll", "type": "106"},
            {"id": 16, "name": "L_Hip_Roll", "type": "106"}, {"id": 13, "name": "R_Hip_Pitch", "type": "106"},
            {"id": 14, "name": "L_Hip_Pitch", "type": "106"}, {"id": 17, "name": "R_Knee_Pitch", "type": "106"},
            {"id": 18, "name": "L_Knee_Pitch", "type": "106"}, {"id": 19, "name": "R_Ankle_Pitch", "type": "106"},
            {"id": 20, "name": "L_Ankle_Pitch", "type": "106"}, {"id": 21, "name": "R_Ankle_Roll", "type": "106"},
            {"id": 22, "name": "L_Ankle_Roll", "type": "106"}
        ]
        
        self.urdf_joint_map = {
            0: "Neck_yaw", 1: "Neck_pitch", 2: "R_Arm_shoulder_yaw", 3: "L_Arm_shoulder_yaw",
            4: "R_Arm_pitch", 5: "L_Arm_pitch", 6: "R_Arm_elbow", 7: "L_Arm_elbow",
            8: "R_arm_hand", 9: "L_Arm_hand", 10: "R_Leg_hip_yaw", 11: "Waist",
            12: "L_Leg_hip_yaw", 15: "R_Leg_hip_roll", 16: "L_Leg_hip_roll",
            13: "R_Leg_hip_pitch", 14: "L_Leg_hip_pitch", 17: "R_Leg_knee",
            18: "L_Leg_knee", 19: "R_Leg_ankle_pitch", 20: "L_Leg_ankle_pitch",
            21: "R_Leg_ankle_roll", 22: "L_Leg_ankle_roll"
        }
        
        self.mirror_map = {
            2: (3, -1), 4: (5, -1), 6: (7, -1), 8: (9, -1), 10: (12, -1), 
            13: (14, -1), 15: (16, -1), 17: (18, -1), 19: (20, -1), 21: (22, -1)  
        }
        
        self.joints = {joint["id"]: 0 for joint in self.joint_data}
        self.sliders = {}
        self.spinboxes = {}
        self.torque_btns = {}
        self.torque_group_btns = {}
        self.torque_groups = {
            "오른팔": [2, 4, 6, 8],
            "왼팔": [3, 5, 7, 9],
            "오른쪽 다리": [10, 13, 15, 17, 19, 21],
            "왼쪽 다리": [12, 14, 16, 18, 20, 22],
        }
        self.online_joints = []
        
        self.load_3d_robot_urdf()
        
        # 🚀 UI를 구성하기 전에, 사용 가능한 포트를 안전하게 먼저 확보합니다.
        print("\n================ [로봇 하드웨어 통신 디버그 로그] ================")
        actual_port = self.find_dynamixel_port()
        
        self.portHandler = PortHandler(actual_port)
        self.packetHandler = SafeProtocol2PacketHandler()
        
        # Read/Write 동기화 객체 추가
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        self.groupSyncWriteTorque = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_TORQUE_ENABLE, 1)

        self.port_opened = False
        if self.portHandler.openPort():
            if self.portHandler.setBaudRate(BAUDRATE):
                self.port_opened = True
                print(f"[✅ 시스템 오픈 완료] 최종 연결 포트: {actual_port}, 속도: {BAUDRATE} bps")
            else:
                print(f"[❌ 보레이트 설정 실패]")
        else:
            print(f"[❌ 통신 포트 오픈 실패]")
            
        self.initUI()
        self.load_persistent_state()
        self.autosave_enabled = True
        
        # 🤖 안전 제일: 포트 연결 성공 시 현재 로봇 포즈부터 스캔하고, 토크는 "OFF" 상태(티칭 모드)로 기동합니다!
        if self.port_opened:
            self.lbl_conn_status.setText(f"🟢 하드웨어 연결 성공 ({actual_port} | {BAUDRATE}bps)")
            self.lbl_conn_status.setStyleSheet("font-weight: bold; font-size: 14pt; background-color: #1e4620; color: #69f0ae; padding: 8px; border-radius: 5px; margin-bottom: 5px;")
            
            # 🚀 현재 실제로 전선에 응답하는 관절 ID만 발라냅니다.
            self.detect_online_joints()
            
            # 🚀 생존한 관절 각도만 SyncRead로 안전하게 긁어와 GUI 및 3D 모델에 매핑시킵니다.
            self.sync_initial_angles()

            # 🚨 로봇이 튀지 않도록 완전히 릴랙스된 티칭(토크 OFF) 상태로 프로그램을 켭니다.
            self.set_all_torque_off()

            # 토크 상태와 무관하게 Present Position을 계속 읽어 프레임 제작
            # 화면의 각도값을 실제 관절 위치로 유지합니다.
            self.live_angle_timer.start()
        else:
            self.lbl_conn_status.setText(f"🔴 연결 실패 (포트/권한 확인)")
            self.lbl_conn_status.setStyleSheet("font-weight: bold; font-size: 14pt; background-color: #5c1d1d; color: #ff5252; padding: 8px; border-radius: 5px; margin-bottom: 5px;")
        print("==================================================================\n")

    # 🚀 [완벽 수정 1] 23개 관절 중 진짜 살아있는 모터 ID를 수색합니다 (오프라인 모터 배제용)
    def detect_online_joints(self):
        self.online_joints = []
        if not self.port_opened: return
        print("📡 [온라인 모터 감지] 23개 관절 중 실제 작동 중인 모터를 수색합니다...")
        for j_id in range(23):
            # 1. 핑 테스트로 확인
            _, dxl_comm_result, _ = self.packetHandler.ping(self.portHandler, j_id)
            if dxl_comm_result == COMM_SUCCESS:
                self.online_joints.append(j_id)
                print(f"   🟢 ID {j_id:02d} - 온라인 확인!")
            else:
                # 2. 백업으로 1바이트 읽기 시도 (간혹 status return level 설정으로 인해 ping을 무시하는 경우 대비)
                _, dxl_comm_result2, _ = self.packetHandler.read1ByteTxRx(self.portHandler, j_id, ADDR_TORQUE_ENABLE)
                if dxl_comm_result2 == COMM_SUCCESS:
                    self.online_joints.append(j_id)
                    print(f"   🟢 ID {j_id:02d} - 온라인 확인! (Read1Byte 백업 성공)")
                else:
                    print(f"   🔴 ID {j_id:02d} - 오프라인 (미응답)")
        print(f"📊 [감지 완료] 총 {len(self.online_joints)}/23 관절이 통신 가능 상태입니다.")

    # 🚀 [완벽 수정 2] 부팅 시 감지된 관절의 실시간 각도를 일괄 매핑시킵니다.
    def sync_initial_angles(self):
        if not self.online_joints: 
            print("   [⚠️ 경고] 생존한 모터가 전혀 감지되지 않아 부팅 초기 각도 로드를 스킵합니다.")
            return
            
        self.groupSyncRead.clearParam()
        for j_id in self.online_joints:
            self.groupSyncRead.addParam(j_id)
            
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result == COMM_SUCCESS:
            print("   [✅ 초기화 스캔] 생존한 물리 관절 각도를 대시보드에 완벽 연동했습니다.")
            for j_id in self.online_joints:
                if self.groupSyncRead.isAvailable(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                    dxl_present_position = self.groupSyncRead.getData(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    angle_deg = self.dxl_position_to_angle(dxl_present_position)
                    self.update_joint_display(j_id, angle_deg, update_robot=False)
            self.update_3d_robot()
        else:
            print(f"   [⚠️ 경고] 기동 시 로봇 각도 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

    def refresh_live_joint_angles(self):
        """토크 ON/OFF와 무관하게 온라인 관절의 실제 각도를 갱신합니다."""
        if not self.port_opened or not self.online_joints:
            return

        self.groupSyncRead.clearParam()
        for j_id in self.online_joints:
            self.groupSyncRead.addParam(j_id)

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.live_angle_read_failures += 1
            # 일시적인 패킷 충돌은 다음 100ms 주기에 복구되므로 로그 폭주를
            # 피하고 연속 실패할 때만 상태를 알립니다.
            if self.live_angle_read_failures == 10:
                print(
                    "[⚠️ 실시간 각도 갱신] 10회 연속 읽기 실패: "
                    f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
                )
            return

        self.live_angle_read_failures = 0
        for j_id in self.online_joints:
            if not self.groupSyncRead.isAvailable(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                continue
            dxl_present_position = self.groupSyncRead.getData(
                j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            angle_deg = self.dxl_position_to_angle(dxl_present_position)
            self.update_joint_display(j_id, angle_deg, update_robot=False)

    def set_joint_connection_ui(self, j_id, is_online, torque_on=False):
        if j_id not in self.torque_btns:
            return

        btn = self.torque_btns[j_id]
        btn.blockSignals(True)
        btn.setEnabled(is_online)
        btn.setChecked(torque_on if is_online else False)
        btn.setText("ON" if is_online and torque_on else "OFF")

        if not is_online:
            btn.setStyleSheet("background-color: #6c757d; color: white; font-weight: bold; border-radius: 4px;")
        elif torque_on:
            btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; border-radius: 4px;")
        else:
            btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")

        if j_id in self.sliders:
            self.sliders[j_id].setEnabled(is_online and torque_on)
        if j_id in self.spinboxes:
            self.spinboxes[j_id].setEnabled(is_online and torque_on)

        btn.blockSignals(False)

    def rescan_online_motors(self):
        print("🔄 [수동 재탐색] 온라인 모터 목록을 다시 스캔합니다...")
        if not self.port_opened:
            QMessageBox.warning(self, "통신 에러", "포트가 열려있지 않아 모터를 재탐색할 수 없습니다.")
            return

        previous_online = set(self.online_joints)
        self.detect_online_joints()
        online_set = set(self.online_joints)
        read_success_ids = []
        torque_on_ids = []

        for j_id in range(23):
            if j_id not in online_set:
                self.set_joint_connection_ui(j_id, False, False)
                continue

            angle_deg = self.read_present_angle(j_id)
            if angle_deg is not None:
                self.update_joint_display(j_id, angle_deg, update_robot=False)
                read_success_ids.append(j_id)

            torque_on = self.read_torque_enabled(j_id)
            if torque_on is None:
                torque_on = self.torque_btns[j_id].isChecked()
            if torque_on:
                torque_on_ids.append(j_id)

            self.set_joint_connection_ui(j_id, True, torque_on)

        if read_success_ids:
            self.update_3d_robot()

        newly_online = sorted(online_set - previous_online)
        lost_ids = sorted(previous_online - online_set)
        self.lbl_conn_status.setText(f"🔄 재탐색 완료: 온라인 {len(online_set)}/23개, 각도 갱신 {len(read_success_ids)}개")
        self.lbl_conn_status.setStyleSheet("font-weight: bold; font-size: 14pt; background-color: #1e4620; color: #69f0ae; padding: 8px; border-radius: 5px; margin-bottom: 5px;" if online_set else "font-weight: bold; font-size: 14pt; background-color: #5c1d1d; color: #ff5252; padding: 8px; border-radius: 5px; margin-bottom: 5px;")

        result_msg = (
            f"온라인 모터: {len(online_set)}/23개\n"
            f"각도 갱신: {len(read_success_ids)}개\n"
            f"토크 ON 감지: {len(torque_on_ids)}개"
        )
        if newly_online:
            result_msg += f"\n새로 연결됨: {newly_online}"
        if lost_ids:
            result_msg += f"\n끊김 감지: {lost_ids}"

        if online_set:
            QMessageBox.information(self, "재탐색 완료", result_msg)
        else:
            QMessageBox.warning(self, "재탐색 완료", result_msg)

    # 🚀 [완벽 수정 3] 실제로 하드웨어에 존재하는 포트만 탐색하여 Fallback 시 에러 및 크래시가 발생하는 것을 원천 차단합니다!
    def find_dynamixel_port(self):
        print("🔍 [자동 포트 탐색] 진짜 U2D2(다이나믹셀)가 연결된 포트를 스캔합니다...")
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']
        existing_ports = [p for p in possible_ports if os.path.exists(p)]
        
        if not existing_ports:
            print("  [❌ 경고] 시스템에 물리적으로 연결된 USB 시리얼 장치(/dev/ttyUSB*)가 전혀 존재하지 않습니다!")
            return DEVICENAME # 기본 포트 반환
            
        for test_port in existing_ports:
            print(f"  👉 테스트 중: {test_port}")
            temp_port = PortHandler(test_port)
            temp_packet = SafeProtocol2PacketHandler()
            
            if temp_port.openPort():
                if temp_port.setBaudRate(BAUDRATE):
                    # 핑 테스트 (0~22번 전 관절 ID 영역 중 단 하나라도 응답하면 유효한 U2D2 버스로 간주)
                    for test_id in range(23):
                        _, dxl_comm_result, _ = temp_packet.ping(temp_port, test_id)
                        if dxl_comm_result == COMM_SUCCESS:
                            print(f"  [🎉 탐색 성공] ID {test_id}번 모터가 응답했습니다! 진짜 U2D2 포트는 {test_port} 입니다.")
                            temp_port.closePort()
                            return test_port
                temp_port.closePort()
                
        # 포트 탐색 및 핑 스캔은 실패했지만, 물리적으로 존재하는 유효한 포트 중 첫 번째 포트를 안전하게 기본 연결 통로로 반환합니다.
        fallback_port = existing_ports[0]
        print(f"  [⚠️ 탐색 실패] 응답하는 모터가 없으나, 시스템에 실존하는 최선의 포트인 {fallback_port}로 크래시 방지용 가상 연결을 수립합니다.")
        return fallback_port

    def closeEvent(self, event):
        self.save_persistent_state()
        if hasattr(self, 'live_angle_timer'):
            self.live_angle_timer.stop()
        if hasattr(self, 'portHandler') and self.portHandler.is_open:
            self.portHandler.closePort()
            print("[✅ 포트 닫힘] 프로그램을 종료하여 포트를 안전하게 반납했습니다.")
        event.accept()

    def normalize_frame_data(self, frame):
        normalized = copy.deepcopy(frame)
        normalized["name"] = str(normalized.get("name", f"Frame {len(self.frames) + 1}"))
        normalized["time_ms"] = int(normalized.get("time_ms", 500))
        normalized["angles"] = self.normalize_angles(normalized.get("angles", {}))
        normalized["torques"] = {int(j): bool(v) for j, v in normalized.get("torques", {}).items()}
        normalized["is_important"] = bool(normalized.get("is_important", False))
        if "start_ms" in normalized:
            normalized["start_ms"] = int(normalized.get("start_ms", 0))
        return normalized

    def rebuild_frame_list_ui(self):
        if not hasattr(self, 'frame_list_ui1'):
            return
        self.frame_list_ui1.clear()
        for frame_data in self.frames:
            item = QListWidgetItem(self.frame_list_ui1)
            custom_widget = FrameItemWidget(frame_data, self)
            item.setSizeHint(custom_widget.sizeHint())
            self.frame_list_ui1.setItemWidget(item, custom_widget)
        self.refresh_library_lists()

    def save_persistent_state(self):
        if not getattr(self, 'autosave_enabled', False):
            return
        try:
            state = {
                "version": 1,
                "frames": self.frames,
                "saved_sequences": self.saved_sequences,
                "motion_sequence": self.motion_sequence,
                "max_seq_ms": self.max_seq_ms,
            }
            with open(self.state_file_path, 'w', encoding='utf-8') as f:
                json.dump(state, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[⚠️ 자동 저장 실패] {type(e).__name__}: {e}")

    def load_persistent_state(self):
        if not os.path.exists(self.state_file_path):
            return
        try:
            with open(self.state_file_path, 'r', encoding='utf-8') as f:
                state = json.load(f)

            self.frames = [self.normalize_frame_data(frame) for frame in state.get("frames", [])]
            for frame in self.frames:
                frame.setdefault("frame_id", uuid.uuid4().hex)
            self.saved_sequences = []
            for seq in state.get("saved_sequences", []):
                self.saved_sequences.append({
                    "name": str(seq.get("name", "Sequence")),
                    "max_seq_ms": int(seq.get("max_seq_ms", 5000)),
                    "repeat_count": max(1, int(seq.get("repeat_count", 1))),
                    "playback_speed": max(0.1, min(5.0, float(seq.get("playback_speed", 1.0)))),
                    "frames": [self.normalize_frame_data(frame) for frame in seq.get("frames", [])],
                })
            self.motion_sequence = [self.normalize_frame_data(frame) for frame in state.get("motion_sequence", [])]
            self.link_sequence_frames_to_library()
            self.max_seq_ms = int(state.get("max_seq_ms", self.max_seq_ms))
            if hasattr(self, 'spin_max_time'):
                self.spin_max_time.setValue(self.max_seq_ms)

            self.rebuild_frame_list_ui()
            self.refresh_sequence_list()
            self.refresh_timeline_ui()
            print(f"[✅ 자동 로드] 프레임 {len(self.frames)}개, 시퀀스 {len(self.saved_sequences)}개를 복원했습니다.")
        except Exception as e:
            print(f"[⚠️ 자동 로드 실패] {type(e).__name__}: {e}")

    def link_sequence_frames_to_library(self):
        """이전 저장 데이터의 시퀀스 프레임을 원본 라이브러리와 연결합니다."""
        by_id = {frame.get("frame_id"): frame for frame in self.frames if frame.get("frame_id")}
        by_name = {}
        for frame in self.frames:
            by_name.setdefault(frame.get("name"), []).append(frame)

        collections = [self.motion_sequence]
        collections.extend(seq.get("frames", []) for seq in self.saved_sequences)
        for frames in collections:
            for sequence_frame in frames:
                frame_id = sequence_frame.get("frame_id")
                if frame_id in by_id:
                    continue
                matches = by_name.get(sequence_frame.get("name"), [])
                if len(matches) == 1:
                    sequence_frame["frame_id"] = matches[0]["frame_id"]

    def load_3d_robot_urdf(self):
        self.urdf_loaded = False
        self.urdf_error = ""
        self.urdf_viewers = []
        self.robot_model = None
        print("[3D 비활성화] URDF 로딩을 건너뜁니다.")

    def init_3d_viewer(self):
        blank = QWidget()
        blank.setMinimumWidth(350)
        blank.setMinimumHeight(300)
        blank.setStyleSheet("background: transparent; border: none;")
        return blank

    def update_3d_robot(self, temp_angles=None):
        return

    def initUI(self):
        self.setWindowTitle('IRC STEP SDK MOTION made by geonwoo')
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(int(screen.width() * 0.95), int(screen.height() * 0.9))
        main_layout = QVBoxLayout()
        self.lbl_conn_status = QLabel("⚠️ 다이나믹셀 하드웨어 포트 연결 대기 중...")
        self.lbl_conn_status.setAlignment(Qt.AlignCenter)
        self.lbl_conn_status.setStyleSheet("font-weight: bold; font-size: 14pt; background-color: #333; color: #FFC107; padding: 8px; border-radius: 5px; margin-bottom: 5px;")
        main_layout.addWidget(self.lbl_conn_status)
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("QTabBar::tab { min-height: 50px; min-width: 420px; font-size: 14pt; font-weight: bold; padding: 10px; }")
        self.tab_frame = QWidget()
        self.init_frame_tab(self.tab_frame)
        self.tab_motion = QWidget()
        self.init_motion_tab(self.tab_motion)
        self.tab_sequence_composer = QWidget()
        self.init_sequence_composer_tab(self.tab_sequence_composer)
        self.tabs.addTab(self.tab_frame, "🎬 1. 단일 프레임 제작 (관절 제어)")
        self.tabs.addTab(self.tab_motion, "🎞️ 2. 모션 제작 (프레임 이어붙이기)")
        self.tabs.addTab(self.tab_sequence_composer, "🧩 3. 시퀀스 조합 (시퀀스 이어붙이기)")
        main_layout.addWidget(self.tabs)
        self.setLayout(main_layout)

    def init_frame_tab(self, tab):
        layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        
        torque_master_layout = QHBoxLayout()
        self.btn_all_torque_on = QPushButton("✅ 전체 토크 ON (잠금)")
        self.btn_all_torque_on.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; font-size: 13pt; min-height: 40px;")
        self.btn_all_torque_on.clicked.connect(self.set_all_torque_on)
        
        self.btn_all_torque_off = QPushButton("❌ 전체 토크 OFF (티칭)")
        self.btn_all_torque_off.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; font-size: 13pt; min-height: 40px;")
        self.btn_all_torque_off.clicked.connect(self.set_all_torque_off)

        self.btn_rescan_motors = QPushButton("🔄 모터 재탐색")
        self.btn_rescan_motors.setStyleSheet("background-color: #17a2b8; color: white; font-weight: bold; font-size: 13pt; min-height: 40px;")
        self.btn_rescan_motors.clicked.connect(self.rescan_online_motors)
        
        torque_master_layout.addWidget(self.btn_all_torque_on)
        torque_master_layout.addWidget(self.btn_all_torque_off)
        torque_master_layout.addWidget(self.btn_rescan_motors)
        left_panel.addLayout(torque_master_layout)

        torque_group_layout = QHBoxLayout()
        for group_name, joint_ids in self.torque_groups.items():
            btn_group = QPushButton(f"{group_name} 전체 OFF")
            btn_group.setCheckable(True)
            btn_group.setMinimumHeight(36)
            btn_group.setStyleSheet(
                "background-color: #dc3545; color: white; font-weight: bold; font-size: 11pt;"
            )
            btn_group.toggled.connect(
                lambda checked, name=group_name, ids=joint_ids: self.sync_torque_group(name, ids, checked)
            )
            self.torque_group_btns[group_name] = btn_group
            torque_group_layout.addWidget(btn_group)
        left_panel.addLayout(torque_group_layout)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        joint_layout = QVBoxLayout()

        for joint in self.joint_data:
            row_frame = QFrame()
            row_frame.setFrameShape(QFrame.StyledPanel)
            row_layout = QHBoxLayout(row_frame)
            lbl_id = QLabel(f"[{joint['id']}]")
            lbl_id.setMinimumWidth(45)
            lbl_id.setStyleSheet("font-weight: bold; font-size: 13pt; color: #0055A4;")
            color = "#FF8C00" if joint["type"] == "28" else "#228B22" if joint["type"] == "64" else "#333333"
            lbl_name = QLabel(f"{joint['name']}\n(MX-{joint['type']})")
            lbl_name.setMinimumWidth(180)
            lbl_name.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 12pt;")
            
            slider = NoWheelSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setMinimumWidth(280) 
            slider.setEnabled(False) 
            
            spinbox = NoWheelSpinBox()
            spinbox.setRange(-180, 180)
            spinbox.setMinimumWidth(70)
            spinbox.setMinimumHeight(40)
            spinbox.setStyleSheet("font-size: 13pt; font-weight: bold;")
            spinbox.setEnabled(False) 
            
            btn_torque = QPushButton("OFF")
            btn_torque.setCheckable(True)
            btn_torque.setChecked(False) # 🚨 디폴트를 OFF(False)로 설정하여 안전하게 기동!
            btn_torque.setMinimumSize(50, 40)
            btn_torque.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold;")
            
            # 🔥 GUI 작동 증명: 람다(lambda)를 통해 완벽하게 연결되어 있습니다.
            btn_torque.toggled.connect(lambda checked, j_id=joint['id']: self.sync_torque(j_id, checked))
            slider.valueChanged.connect(lambda val, j_id=joint['id']: self.sync_values(j_id, val, 'slider'))
            # 숫자를 입력하는 동안에는 GUI만 갱신하고, Enter를 눌렀을 때만
            # 최종 값을 모터로 전송합니다.
            spinbox.valueChanged.connect(
                lambda val, j_id=joint['id']: self.sync_values(j_id, val, 'spinbox', transmit_motor=False)
            )
            spinbox.lineEdit().returnPressed.connect(
                lambda j_id=joint['id'], widget=spinbox: self.sync_values(
                    j_id, widget.value(), 'spinbox', transmit_motor=True
                )
            )
            
            self.sliders[joint['id']] = slider
            self.spinboxes[joint['id']] = spinbox
            self.torque_btns[joint['id']] = btn_torque
            
            row_layout.addWidget(lbl_id)
            row_layout.addWidget(lbl_name)
            row_layout.addWidget(slider, stretch=1)
            row_layout.addWidget(spinbox)
            row_layout.addWidget(btn_torque)
            joint_layout.addWidget(row_frame)
        
        scroll_content.setLayout(joint_layout)
        scroll_area.setWidget(scroll_content)
        left_panel.addWidget(scroll_area)
        layout.addLayout(left_panel, 2) 

        sim_group = QGroupBox("")
        sim_group.setStyleSheet("border: none;")
        sim_layout = QVBoxLayout()
        self.canvas_3d = self.init_3d_viewer()
        sim_layout.addWidget(self.canvas_3d)
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group, 2)

        frame_group = QGroupBox("저장된 프레임 목록")
        frame_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        frame_layout = QVBoxLayout()

        self.btn_read_robot = QPushButton('🤖 물리 로봇 실제 관절값 불러오기\n(토크 OFF 후 손으로 꺾고 누르세요)')
        self.btn_read_robot.setStyleSheet("background-color: #FFC107; color: #333; font-weight: bold; font-size: 13pt; min-height: 50px;")
        self.btn_read_robot.clicked.connect(self.read_angles_from_robot)
        frame_layout.addWidget(self.btn_read_robot)

        time_layout = QHBoxLayout()
        time_label = QLabel("이동 시간(ms):")
        self.time_spinbox = QSpinBox()
        self.time_spinbox.setRange(10, 5000)
        self.time_spinbox.setValue(500)
        self.time_spinbox.setSingleStep(50)
        self.time_spinbox.setMinimumHeight(45)
        time_layout.addWidget(time_label)
        time_layout.addWidget(self.time_spinbox)
        frame_layout.addLayout(time_layout)

        self.btn_add = QPushButton('+ 프레임 추가')
        self.btn_add.setStyleSheet("background-color: #E6F0FA; font-weight: bold; font-size: 14pt; min-height: 50px;")
        self.btn_add.clicked.connect(self.add_frame)
        frame_layout.addWidget(self.btn_add)

        self.btn_load_frame = QPushButton('📂 프레임 불러오기 (각도 바로 적용)')
        self.btn_load_frame.setStyleSheet(
            "background-color: #007bff; color: white; font-weight: bold; font-size: 14pt; min-height: 50px;"
        )
        self.btn_load_frame.clicked.connect(self.apply_selected_frame)
        frame_layout.addWidget(self.btn_load_frame)

        self.btn_execute = QPushButton('▶️ 단일 실행')
        self.btn_execute.setStyleSheet("background-color: #FF5722; color: white; font-weight: bold; font-size: 14pt; min-height: 50px;")
        self.btn_execute.clicked.connect(self.execute_frame)
        frame_layout.addWidget(self.btn_execute)
        
        grid_btn_layout = QGridLayout()
        btn_style = "font-size: 12pt; min-height: 40px; font-weight: bold;"
        
        self.btn_update = QPushButton('💾 재저장')
        self.btn_update.setStyleSheet(btn_style)
        self.btn_update.clicked.connect(self.update_frame)
        self.btn_rename = QPushButton('✏️ 이름 변경')
        self.btn_rename.setStyleSheet(btn_style)
        self.btn_rename.clicked.connect(self.rename_frame)
        self.btn_mirror = QPushButton('🔄 좌우반전')
        self.btn_mirror.setStyleSheet(btn_style)
        self.btn_mirror.clicked.connect(self.mirror_frame)
        
        delete_layout = QHBoxLayout()
        self.btn_toggle_select = QPushButton('☑️ 선택')
        self.btn_toggle_select.setCheckable(True)
        self.btn_toggle_select.setStyleSheet(btn_style)
        self.btn_toggle_select.toggled.connect(self.toggle_select_mode)
        self.btn_delete = QPushButton('🗑️ 삭제')
        self.btn_delete.setStyleSheet(btn_style)
        self.btn_delete.clicked.connect(self.delete_frame)
        delete_layout.addWidget(self.btn_toggle_select)
        delete_layout.addWidget(self.btn_delete)
        
        grid_btn_layout.addWidget(self.btn_update, 0, 0)
        grid_btn_layout.addWidget(self.btn_rename, 0, 1)
        grid_btn_layout.addWidget(self.btn_mirror, 1, 0)
        grid_btn_layout.addLayout(delete_layout, 1, 1)
        frame_layout.addLayout(grid_btn_layout)

        self.frame_list_ui1 = QListWidget()
        self.frame_list_ui1.itemSelectionChanged.connect(self.sync_drag_selection)
        self.frame_list_ui1.currentRowChanged.connect(self.load_frame_to_ui)
        frame_layout.addWidget(self.frame_list_ui1)

        frame_group.setLayout(frame_layout)
        layout.addWidget(frame_group, 1)
        tab.setLayout(layout)

    def init_motion_tab(self, tab):
        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        
        lib_group = QGroupBox("📚 라이브러리 (드래그 앤 드롭 가능!)")
        lib_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        lib_layout = QHBoxLayout() 
        
        sequence_layout = QVBoxLayout()
        lbl_sequence = QLabel("🎞️ 저장된 시퀀스")
        lbl_sequence.setStyleSheet("color: #6f42c1;")
        
        self.sequence_list_ui = QListWidget()
        self.sequence_list_ui.setSelectionMode(QAbstractItemView.SingleSelection)
        self.sequence_list_ui.itemDoubleClicked.connect(self.load_sequence_from_list_item)
        
        sequence_layout.addWidget(lbl_sequence)
        self.lbl_loaded_sequence = QLabel("현재 타임라인: 새 시퀀스")
        self.lbl_loaded_sequence.setStyleSheet("color: #00897b; font-weight: bold;")
        sequence_layout.addWidget(self.lbl_loaded_sequence)
        sequence_layout.addWidget(self.sequence_list_ui)
        self.btn_load_saved_sequence = QPushButton("📥 선택 시퀀스를 타임라인에 불러오기")
        self.btn_load_saved_sequence.setStyleSheet(
            "background-color: #6f42c1; color: white; font-weight: bold;"
        )
        self.btn_load_saved_sequence.clicked.connect(self.load_selected_sequence_to_timeline)
        sequence_layout.addWidget(self.btn_load_saved_sequence)
        self.btn_delete_saved_sequence = QPushButton("🗑️ 선택 시퀀스 삭제")
        self.btn_delete_saved_sequence.setStyleSheet(
            "background-color: #dc3545; color: white; font-weight: bold;"
        )
        self.btn_delete_saved_sequence.clicked.connect(self.delete_selected_saved_sequence)
        sequence_layout.addWidget(self.btn_delete_saved_sequence)
        
        all_layout = QVBoxLayout()
        lbl_all = QLabel("📁 전체 프레임")
        self.frame_list_all = QListWidget()
        self.frame_list_all.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.frame_list_all.setDragEnabled(True)
        self.frame_list_all.itemSelectionChanged.connect(self.on_all_selected)
        
        all_layout.addWidget(lbl_all)
        all_layout.addWidget(self.frame_list_all)

        lib_layout.addLayout(sequence_layout)
        lib_layout.addLayout(all_layout)
        lib_group.setLayout(lib_layout)
        
        mid_layout = QVBoxLayout()
        mid_layout.setAlignment(Qt.AlignCenter)
        self.btn_add_to_motion = QPushButton('타임라인에\n추가 ⬇️')
        self.btn_add_to_motion.setMinimumSize(100, 70)
        self.btn_add_to_motion.setStyleSheet("font-weight: bold; font-size: 13pt; background-color: #007bff; color: white;")
        self.btn_add_to_motion.clicked.connect(self.add_to_motion)
        mid_layout.addWidget(self.btn_add_to_motion)
        
        sim_group_tab2 = QGroupBox("")
        sim_group_tab2.setStyleSheet("border: none;")
        sim_layout_tab2 = QVBoxLayout()
        self.canvas_3d_tab2 = self.init_3d_viewer() 
        sim_layout_tab2.addWidget(self.canvas_3d_tab2)
        sim_group_tab2.setLayout(sim_layout_tab2)

        top_layout.addWidget(lib_group, 4)
        top_layout.addLayout(mid_layout, 1)
        top_layout.addWidget(sim_group_tab2, 4)
        
        ctrl_group = QGroupBox("⚙️ 시퀀스 컨트롤 패널")
        ctrl_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        ctrl_layout = QHBoxLayout()
        
        self.btn_keyframe_play = QPushButton('▶️ 재생')
        self.btn_keyframe_play.setStyleSheet("background-color: #ff9800; color: white; min-height: 40px; font-weight: bold;")
        self.btn_keyframe_play.clicked.connect(self.play_motion_page_sequence)

        self.btn_keyframe_pause = QPushButton('⏸️ 일시정지')
        self.btn_keyframe_pause.setStyleSheet("background-color: #607d8b; color: white; min-height: 40px; font-weight: bold;")
        self.btn_keyframe_pause.clicked.connect(self.pause_motion_sequence)
        self.btn_keyframe_pause.setEnabled(False)

        self.btn_stop_motion = QPushButton('⏹️ 정지')
        self.btn_stop_motion.setStyleSheet("background-color: #dc3545; color: white; min-height: 40px;")
        self.btn_stop_motion.clicked.connect(self.stop_motion_sequence)
        self.btn_stop_motion.setEnabled(False)

        self.btn_robot_sync = QPushButton('🤖 로봇 동기화: OFF')
        self.btn_robot_sync.setCheckable(True)
        self.btn_robot_sync.setStyleSheet(
            "QPushButton { background-color: #555; color: white; min-height: 40px; font-weight: bold; }"
            "QPushButton:checked { background-color: #e91e63; }"
        )
        self.btn_robot_sync.toggled.connect(self.toggle_robot_sync)

        repeat_label = QLabel("반복:")
        self.spin_motion_repeat = QSpinBox()
        self.spin_motion_repeat.setRange(1, 999)
        self.spin_motion_repeat.setValue(1)
        self.spin_motion_repeat.setSuffix(" 회")
        self.spin_motion_repeat.setMinimumWidth(85)
        speed_label = QLabel("배속:")
        self.spin_motion_speed = QDoubleSpinBox()
        self.spin_motion_speed.setRange(0.1, 5.0)
        self.spin_motion_speed.setDecimals(1)
        self.spin_motion_speed.setSingleStep(0.1)
        self.spin_motion_speed.setValue(1.0)
        self.spin_motion_speed.setSuffix("x")
        self.spin_motion_speed.setMinimumWidth(90)

        self.btn_default_pose = QPushButton('🏠 기본자세 복귀 (천천히)')
        self.btn_default_pose.setStyleSheet(
            "background-color: #6f42c1; color: white; min-height: 40px; font-weight: bold;"
        )
        self.btn_default_pose.clicked.connect(self.return_to_default_pose)

        self.btn_clear_motion = QPushButton('🧹 초기화')
        self.btn_clear_motion.clicked.connect(self.clear_motion)
        self.btn_save_sequence = QPushButton('💾 저장')
        self.btn_save_sequence.setStyleSheet("background-color: #17a2b8; color: white;")
        self.btn_save_sequence.clicked.connect(self.save_sequence)
        self.btn_manage_seq = QPushButton('📁 관리/불러오기')
        self.btn_manage_seq.clicked.connect(self.open_sequence_manager)
        self.btn_export = QPushButton('🚀 Jetson 내보내기')
        self.btn_export.setStyleSheet("background-color: #28a745; color: white;")
        self.btn_export.clicked.connect(self.export_motion_json)
        
        ctrl_layout.addWidget(self.btn_keyframe_play)
        ctrl_layout.addWidget(self.btn_keyframe_pause)
        ctrl_layout.addWidget(self.btn_stop_motion)
        ctrl_layout.addWidget(self.btn_robot_sync)
        ctrl_layout.addWidget(repeat_label)
        ctrl_layout.addWidget(self.spin_motion_repeat)
        ctrl_layout.addWidget(speed_label)
        ctrl_layout.addWidget(self.spin_motion_speed)
        ctrl_layout.addWidget(self.btn_default_pose)
        ctrl_layout.addSpacing(15)
        ctrl_layout.addWidget(self.btn_clear_motion)
        ctrl_layout.addWidget(self.btn_save_sequence)
        ctrl_layout.addWidget(self.btn_manage_seq)
        ctrl_layout.addSpacing(15)
        ctrl_layout.addWidget(self.btn_export)
        ctrl_group.setLayout(ctrl_layout)

        timeline_group = QGroupBox("🎬 프레임 타임라인 (배경 드래그: 시간탐색 / 라이브러리에서 끌어놓기 가능!)")
        timeline_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        tl_base_layout = QVBoxLayout()
        
        tl_tools_layout = QHBoxLayout()
        self.lbl_total_time = QLabel("현재 모션 종료 지점: 0ms / 타임라인 길이: 5000ms")
        self.lbl_total_time.setStyleSheet("color: #69f0ae;")
        
        bright_spinbox_style = "background-color: #ffffff; color: #000000; font-weight: bold; border: 1px solid #aaa; font-size: 12pt;"
        
        lbl_max_time = QLabel("⏳ 타임라인 총 길이 설정(ms):")
        self.spin_max_time = QSpinBox()
        self.spin_max_time.setRange(1000, 60000)
        self.spin_max_time.setValue(self.max_seq_ms)
        self.spin_max_time.setSingleStep(500)
        self.spin_max_time.setStyleSheet(bright_spinbox_style)

        lbl_zoom = QLabel("   |   타임라인 확대(%):")
        self.spin_timeline_zoom = QSpinBox()
        self.spin_timeline_zoom.setRange(25, 400)
        self.spin_timeline_zoom.setSingleStep(25)
        self.spin_timeline_zoom.setValue(int(self.SCALE * 100))
        self.spin_timeline_zoom.setSuffix(" %")
        self.spin_timeline_zoom.setStyleSheet(bright_spinbox_style)
        self.spin_timeline_zoom.valueChanged.connect(self.apply_timeline_zoom)
        
        self.btn_apply_max_time = QPushButton("적용")
        self.btn_apply_max_time.setStyleSheet("background-color: #6f42c1; color: white;")
        self.btn_apply_max_time.clicked.connect(self.apply_max_sequence_time)
        
        lbl_target = QLabel("   |   시간 일괄 스케일링(ms):")
        self.spin_target_time = QSpinBox()
        self.spin_target_time.setRange(10, 60000)
        self.spin_target_time.setValue(1000)
        self.spin_target_time.setStyleSheet(bright_spinbox_style)
        
        self.btn_apply_time = QPushButton("비율 적용")
        self.btn_apply_time.setStyleSheet("background-color: #6f42c1; color: white;")
        self.btn_apply_time.clicked.connect(self.apply_target_time)
        
        tl_tools_layout.addWidget(self.lbl_total_time)
        tl_tools_layout.addStretch()
        tl_tools_layout.addWidget(lbl_max_time)
        tl_tools_layout.addWidget(self.spin_max_time)
        tl_tools_layout.addWidget(self.btn_apply_max_time)
        tl_tools_layout.addWidget(lbl_zoom)
        tl_tools_layout.addWidget(self.spin_timeline_zoom)
        tl_tools_layout.addWidget(lbl_target)
        tl_tools_layout.addWidget(self.spin_target_time)
        tl_tools_layout.addWidget(self.btn_apply_time)
        tl_base_layout.addLayout(tl_tools_layout)

        gap_tools_layout = QHBoxLayout()
        gap_tools_layout.addStretch()
        gap_label = QLabel("↔️ 모든 프레임 사이 동일 간격(ms):")
        self.spin_uniform_frame_gap = QSpinBox()
        self.spin_uniform_frame_gap.setRange(0, 10000)
        self.spin_uniform_frame_gap.setSingleStep(10)
        self.spin_uniform_frame_gap.setValue(0)
        self.spin_uniform_frame_gap.setSuffix(" ms")
        self.spin_uniform_frame_gap.setStyleSheet(bright_spinbox_style)
        self.spin_uniform_frame_gap.lineEdit().returnPressed.connect(self.apply_uniform_frame_gap)
        self.btn_apply_uniform_gap = QPushButton("간격 일괄 적용")
        self.btn_apply_uniform_gap.setStyleSheet("background-color: #00897b; color: white;")
        self.btn_apply_uniform_gap.clicked.connect(self.apply_uniform_frame_gap)
        gap_tools_layout.addWidget(gap_label)
        gap_tools_layout.addWidget(self.spin_uniform_frame_gap)
        gap_tools_layout.addWidget(self.btn_apply_uniform_gap)
        tl_base_layout.addLayout(gap_tools_layout)
        
        self.timeline_scroll = QScrollArea()
        self.timeline_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.timeline_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.timeline_scroll.setMinimumHeight(200)
        
        self.timeline_container = TimelineContainer(self)
        self.timeline_scroll.setWidget(self.timeline_container)
        
        tl_base_layout.addWidget(self.timeline_scroll)
        timeline_group.setLayout(tl_base_layout)
        
        main_layout.addLayout(top_layout, 5)
        main_layout.addWidget(ctrl_group, 1)
        main_layout.addWidget(timeline_group, 3)
        
        tab.setLayout(main_layout)

    def init_sequence_composer_tab(self, tab):
        self.COMPOSER_SCALE = 1.0
        self.sequence_composer_entries = []
        main_layout = QVBoxLayout(tab)

        title = QLabel("🧩 저장된 시퀀스를 이어붙여 새로운 시퀀스 만들기")
        title.setStyleSheet("font-size: 16pt; font-weight: bold; padding: 8px;")
        main_layout.addWidget(title)

        source_group = QGroupBox("저장된 시퀀스")
        source_layout = QVBoxLayout(source_group)
        self.composer_source_list = QListWidget()
        self.composer_source_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.composer_source_list.setDragEnabled(True)
        self.composer_source_list.itemDoubleClicked.connect(lambda _: self.add_sequence_to_composer())
        source_layout.addWidget(self.composer_source_list)

        add_button = QPushButton("선택 시퀀스 추가 ➡️")
        add_button.setMinimumHeight(45)
        add_button.setStyleSheet("background-color: #007bff; color: white; font-weight: bold;")
        add_button.clicked.connect(self.add_sequence_to_composer)
        source_layout.addWidget(add_button)
        delete_source_button = QPushButton("🗑️ 선택 시퀀스 삭제")
        delete_source_button.setStyleSheet(
            "background-color: #dc3545; color: white; font-weight: bold;"
        )
        delete_source_button.clicked.connect(self.delete_composer_selected_sequence)
        source_layout.addWidget(delete_source_button)

        chain_group = QGroupBox("시퀀스 타임라인 (블록 드래그로 순서 변경 / 저장 시퀀스 끌어놓기)")
        chain_layout = QVBoxLayout(chain_group)
        composer_tools = QHBoxLayout()
        self.lbl_composer_meta = QLabel("구성: 0개 시퀀스 / 0개 프레임 / 0ms")
        self.lbl_composer_meta.setStyleSheet(
            "font-size: 13pt; font-weight: bold; color: #69f0ae; padding: 8px;"
        )
        composer_zoom_label = QLabel("타임라인 확대:")
        self.spin_composer_zoom = QSpinBox()
        self.spin_composer_zoom.setRange(25, 400)
        self.spin_composer_zoom.setSingleStep(25)
        self.spin_composer_zoom.setValue(100)
        self.spin_composer_zoom.setSuffix(" %")
        self.spin_composer_zoom.valueChanged.connect(self.apply_composer_timeline_zoom)
        composer_tools.addWidget(self.lbl_composer_meta)
        composer_tools.addStretch()
        composer_tools.addWidget(composer_zoom_label)
        composer_tools.addWidget(self.spin_composer_zoom)
        chain_layout.addLayout(composer_tools)
        self.composer_timeline_scroll = QScrollArea()
        self.composer_timeline_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.composer_timeline_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.composer_timeline_scroll.setMinimumHeight(180)
        self.composer_timeline_container = SequenceTimelineContainer(self)
        self.composer_timeline_scroll.setWidget(self.composer_timeline_container)
        chain_layout.addWidget(self.composer_timeline_scroll)
        main_layout.addWidget(source_group, 2)

        composer_ctrl_group = QGroupBox("⚙️ 시퀀스 컨트롤 패널")
        composer_ctrl_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        controls = QHBoxLayout(composer_ctrl_group)
        self.composer_btn_play = QPushButton("▶️ 재생")
        self.composer_btn_play.clicked.connect(self.play_composed_sequence)
        self.composer_btn_pause = QPushButton("⏸️ 일시정지")
        self.composer_btn_pause.clicked.connect(self.pause_motion_sequence)
        self.composer_btn_pause.setEnabled(False)
        self.composer_btn_stop = QPushButton("⏹️ 정지")
        self.composer_btn_stop.clicked.connect(self.stop_motion_sequence)
        self.composer_btn_stop.setEnabled(False)
        self.composer_btn_sync = QPushButton("🤖 로봇 동기화: OFF")
        self.composer_btn_sync.setCheckable(True)
        self.composer_btn_sync.toggled.connect(self.toggle_composer_robot_sync)
        composer_repeat_label = QLabel("반복:")
        self.spin_composer_repeat = QSpinBox()
        self.spin_composer_repeat.setRange(1, 999)
        self.spin_composer_repeat.setValue(1)
        self.spin_composer_repeat.setSuffix(" 회")
        self.spin_composer_repeat.setMinimumWidth(85)
        composer_speed_label = QLabel("배속:")
        self.spin_composer_speed = QDoubleSpinBox()
        self.spin_composer_speed.setRange(0.1, 5.0)
        self.spin_composer_speed.setDecimals(1)
        self.spin_composer_speed.setSingleStep(0.1)
        self.spin_composer_speed.setValue(1.0)
        self.spin_composer_speed.setSuffix("x")
        self.spin_composer_speed.setMinimumWidth(90)
        default_button = QPushButton("🏠 기본자세 복귀")
        default_button.clicked.connect(self.return_to_default_pose)
        clear_button = QPushButton("🧹 초기화")
        clear_button.clicked.connect(self.clear_sequence_composer)
        save_button = QPushButton("💾 저장")
        save_button.clicked.connect(self.save_composed_sequence)
        manage_button = QPushButton("📁 관리/불러오기")
        manage_button.clicked.connect(self.open_sequence_manager)
        export_button = QPushButton("🚀 Jetson 내보내기")
        export_button.clicked.connect(self.export_composed_sequence_json)
        for button in (
            self.composer_btn_play, self.composer_btn_pause, self.composer_btn_stop,
            self.composer_btn_sync,
        ):
            button.setMinimumHeight(45)
            controls.addWidget(button)
        controls.addWidget(composer_repeat_label)
        controls.addWidget(self.spin_composer_repeat)
        controls.addWidget(composer_speed_label)
        controls.addWidget(self.spin_composer_speed)
        for button in (default_button, clear_button, save_button, manage_button, export_button):
            button.setMinimumHeight(45)
            controls.addWidget(button)
        main_layout.addWidget(composer_ctrl_group, 1)
        main_layout.addWidget(chain_group, 3)

    def refresh_composer_source_list(self):
        if not hasattr(self, 'composer_source_list'):
            return
        self.composer_source_list.clear()
        for idx, sequence in enumerate(self.saved_sequences):
            frames = sequence.get('frames', [])
            duration = max(
                (frame.get('start_ms', 0) + frame.get('time_ms', 0) for frame in frames),
                default=0,
            )
            item = QListWidgetItem(
                f"[{idx + 1}] {sequence['name']} "
                f"({len(frames)} 프레임 / {duration}ms / "
                f"{sequence.get('repeat_count', 1)}회 / {sequence.get('playback_speed', 1.0):.1f}x)"
            )
            item.setData(Qt.UserRole, idx)
            self.composer_source_list.addItem(item)
        if hasattr(self, 'sequence_composer_entries'):
            for entry in self.sequence_composer_entries:
                idx = entry['sequence_idx']
                if 0 <= idx < len(self.saved_sequences):
                    sequence = self.saved_sequences[idx]
                    entry['name'] = sequence['name']
                    entry['time_ms'] = self.sequence_duration(idx)
                    entry['frame_count'] = len(sequence.get('frames', []))
            self.pack_sequence_composer_entries()
            self.refresh_sequence_composer_timeline()

    def sequence_duration(self, sequence_idx):
        frames = self.saved_sequences[sequence_idx].get('frames', [])
        if not frames:
            return 0
        start = min(frame.get('start_ms', 0) for frame in frames)
        end = max(frame.get('start_ms', 0) + frame.get('time_ms', 0) for frame in frames)
        return max(0, end - start)

    def add_sequence_to_composer(self, sequence_idx=None, drop_x=None):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        if sequence_idx is None:
            item = self.composer_source_list.currentItem()
            if item is None:
                return QMessageBox.warning(self, "경고", "추가할 저장 시퀀스를 선택하세요.")
            sequence_idx = item.data(Qt.UserRole)
        sequence = self.saved_sequences[sequence_idx]
        duration = self.sequence_duration(sequence_idx)
        entry = {
            "sequence_idx": sequence_idx,
            "name": sequence['name'],
            "time_ms": duration,
            "frame_count": len(sequence.get('frames', [])),
            "start_ms": self.composer_total_duration(),
        }
        self.sequence_composer_entries.append(entry)
        if drop_x is not None:
            self.reorder_sequence_composer_entry(entry, drop_x)
        else:
            self.pack_sequence_composer_entries()
            self.refresh_sequence_composer_timeline()

    def remove_sequence_composer_entry(self, index):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        if 0 <= index < len(self.sequence_composer_entries):
            self.sequence_composer_entries.pop(index)
            self.pack_sequence_composer_entries()
            self.refresh_sequence_composer_timeline()

    def clear_sequence_composer(self):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        self.sequence_composer_entries.clear()
        self.refresh_sequence_composer_timeline()

    def composer_sequence_indices(self):
        return [entry['sequence_idx'] for entry in self.sequence_composer_entries]

    def composer_total_duration(self):
        return sum(entry['time_ms'] for entry in self.sequence_composer_entries)

    def composer_timeline_width(self):
        return max(1200, int(self.composer_total_duration() * self.COMPOSER_SCALE) + 10)

    def pack_sequence_composer_entries(self):
        current = 0
        for entry in self.sequence_composer_entries:
            entry['start_ms'] = current
            current += entry['time_ms']

    def reorder_sequence_composer_entry(self, moved_entry, moved_x):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        others = [entry for entry in self.sequence_composer_entries if entry is not moved_entry]
        moved_center = moved_x + moved_entry['time_ms'] * self.COMPOSER_SCALE / 2.0
        insert_at = 0
        for entry in others:
            center = (entry['start_ms'] + entry['time_ms'] / 2.0) * self.COMPOSER_SCALE
            if moved_center >= center:
                insert_at += 1
        others.insert(insert_at, moved_entry)
        self.sequence_composer_entries = others
        self.pack_sequence_composer_entries()
        self.refresh_sequence_composer_timeline()

    def refresh_sequence_composer_timeline(self):
        if not hasattr(self, 'composer_timeline_container'):
            return
        for child in self.composer_timeline_container.findChildren(SequenceTimelineBlockWidget):
            child.setParent(None)
            child.deleteLater()
        for index, entry in enumerate(self.sequence_composer_entries):
            block = SequenceTimelineBlockWidget(entry, index, self)
            block.setParent(self.composer_timeline_container)
            x = int(entry['start_ms'] * self.COMPOSER_SCALE)
            width = max(1, int(entry['time_ms'] * self.COMPOSER_SCALE))
            block.setGeometry(x, 40, width, 90)
            block.show()
        self.composer_timeline_container.setMinimumSize(self.composer_timeline_width(), 145)
        self.composer_timeline_container.update()
        self.refresh_composer_meta()

    def build_composed_sequence_frames(self):
        combined_frames = []
        current_start = 0
        for sequence_idx in self.composer_sequence_indices():
            source_frames = self.saved_sequences[sequence_idx].get('frames', [])
            if not source_frames:
                continue
            ordered = sorted(source_frames, key=lambda frame: frame.get('start_ms', 0))
            source_start = min(frame.get('start_ms', 0) for frame in ordered)
            source_end = max(
                frame.get('start_ms', 0) + max(MIN_TIMELINE_FRAME_MS, frame.get('time_ms', 0))
                for frame in ordered
            )
            for source_frame in ordered:
                new_frame = copy.deepcopy(source_frame)
                new_frame['time_ms'] = max(MIN_TIMELINE_FRAME_MS, new_frame['time_ms'])
                new_frame['start_ms'] = current_start + source_frame.get('start_ms', 0) - source_start
                combined_frames.append(new_frame)
            current_start += source_end - source_start
        return combined_frames, current_start

    def refresh_composer_meta(self, *args):
        if not hasattr(self, 'lbl_composer_meta'):
            return
        frames, duration = self.build_composed_sequence_frames()
        self.lbl_composer_meta.setText(
            f"구성: {len(self.sequence_composer_entries)}개 시퀀스 / {len(frames)}개 프레임 / {duration}ms"
        )

    def save_composed_sequence(self):
        if not self.sequence_composer_entries:
            return QMessageBox.warning(self, "경고", "먼저 조합할 시퀀스를 추가하세요.")
        frames, duration = self.build_composed_sequence_frames()
        if not frames:
            return QMessageBox.warning(self, "경고", "조합 결과에 프레임이 없습니다.")
        if duration > 60000:
            return QMessageBox.warning(self, "시간 초과", "조합 결과가 최대 타임라인 길이 60000ms를 초과합니다.")

        name, ok = QInputDialog.getText(self, "시퀀스 조합 저장", "새 시퀀스 이름:")
        if not ok or not name.strip():
            return
        self.saved_sequences.append({
            "name": name.strip(),
            "max_seq_ms": max(1000, duration),
            "repeat_count": self.spin_composer_repeat.value(),
            "playback_speed": self.spin_composer_speed.value(),
            "frames": frames,
        })
        self.refresh_sequence_list()
        self.save_persistent_state()
        QMessageBox.information(
            self,
            "저장 완료",
            f"'{name.strip()}' 시퀀스를 {len(frames)}개 프레임, {duration}ms로 저장했습니다.",
        )

    def activate_composer_playback(self):
        frames, duration = self.build_composed_sequence_frames()
        if not frames:
            QMessageBox.warning(self, "경고", "재생할 조합 시퀀스가 없습니다.")
            return False
        if self.playback_context != "composer":
            self.composer_motion_backup = (self.motion_sequence, self.max_seq_ms)
        self.motion_sequence = copy.deepcopy(frames)
        self.max_seq_ms = max(1, duration)
        self.playback_context = "composer"
        return True

    def play_composed_sequence(self):
        if self.playback_context != "composer" and not self.activate_composer_playback():
            return
        if not self.is_paused:
            self.playback_repeat_target = self.spin_composer_repeat.value()
            self.playback_repeat_current = 1
            self.playback_speed = self.spin_composer_speed.value()
        self.play_keyframe_sequence()

    def toggle_composer_robot_sync(self, checked):
        if checked:
            if not self.sequence_composer_entries:
                self.composer_btn_sync.blockSignals(True)
                self.composer_btn_sync.setChecked(False)
                self.composer_btn_sync.blockSignals(False)
                return QMessageBox.warning(self, "경고", "먼저 조합할 시퀀스를 추가하세요.")
            if self.robot_sync_enabled:
                self.composer_btn_sync.setText("🤖 로봇 동기화: ON")
                return
            original_motion, original_max = self.motion_sequence, self.max_seq_ms
            frames, duration = self.build_composed_sequence_frames()
            self.motion_sequence, self.max_seq_ms = frames, max(1, duration)
            self.btn_robot_sync.setChecked(True)
            self.motion_sequence, self.max_seq_ms = original_motion, original_max
            if not self.robot_sync_enabled:
                self.composer_btn_sync.blockSignals(True)
                self.composer_btn_sync.setChecked(False)
                self.composer_btn_sync.blockSignals(False)
            else:
                self.composer_btn_sync.setText("🤖 로봇 동기화: ON")
        else:
            if self.robot_sync_enabled:
                self.btn_robot_sync.setChecked(False)
            self.composer_btn_sync.setText("🤖 로봇 동기화: OFF")

    def export_composed_sequence_json(self):
        frames, duration = self.build_composed_sequence_frames()
        if not frames:
            return QMessageBox.warning(self, "경고", "내보낼 조합 시퀀스가 없습니다.")
        file_name, _ = QFileDialog.getSaveFileName(
            self, "조합 시퀀스 저장", "jetson_composed_sequence.json", "JSON Files (*.json)"
        )
        if file_name:
            with open(file_name, 'w', encoding='utf-8') as file:
                json.dump({
                    "max_seq_ms": duration,
                    "repeat_count": self.spin_composer_repeat.value(),
                    "playback_speed": self.spin_composer_speed.value(),
                    "frames": frames,
                }, file, indent=4, ensure_ascii=False)
            QMessageBox.information(self, "성공", "조합 시퀀스 데이터 추출 완료!")

    def apply_timeline_zoom(self, value):
        old_scale = self.SCALE
        viewport_center_ms = (
            self.timeline_scroll.horizontalScrollBar().value()
            + self.timeline_scroll.viewport().width() / 2
        ) / old_scale
        self.SCALE = value / 100.0
        self.refresh_timeline_ui()
        new_scroll = int(viewport_center_ms * self.SCALE - self.timeline_scroll.viewport().width() / 2)
        self.timeline_scroll.horizontalScrollBar().setValue(max(0, new_scroll))

    def apply_composer_timeline_zoom(self, value):
        old_scale = self.COMPOSER_SCALE
        center_ms = (
            self.composer_timeline_scroll.horizontalScrollBar().value()
            + self.composer_timeline_scroll.viewport().width() / 2
        ) / old_scale
        self.COMPOSER_SCALE = value / 100.0
        self.refresh_sequence_composer_timeline()
        new_scroll = int(
            center_ms * self.COMPOSER_SCALE
            - self.composer_timeline_scroll.viewport().width() / 2
        )
        self.composer_timeline_scroll.horizontalScrollBar().setValue(max(0, new_scroll))

    def previous_frame_end_ms(self, target_frame):
        ordered = sorted(self.motion_sequence, key=lambda frame: frame['start_ms'])
        index = next(
            (idx for idx, frame in enumerate(ordered) if frame is target_frame),
            -1,
        )
        if index <= 0:
            return 0
        previous = ordered[index - 1]
        return previous['start_ms'] + previous['time_ms']

    def frame_gap_ms(self, target_frame):
        return max(0, target_frame['start_ms'] - self.previous_frame_end_ms(target_frame))

    def reorder_motion_frame(self, moved_frame, moved_x):
        """빈 위치는 유지하고, 충돌 위치 삽입 시 뒤 블록만 필요한 만큼 밉니다."""
        duration = max(MIN_TIMELINE_FRAME_MS, int(moved_frame['time_ms']))
        max_start = max(0, self.max_seq_ms - duration)
        desired_start = max(0, min(int(round(moved_x / self.SCALE)), max_start))
        others = [frame for frame in self.motion_sequence if frame is not moved_frame]
        original_positions = [(frame, frame['start_ms']) for frame in self.motion_sequence]

        def is_free(start_ms):
            end_ms = start_ms + duration
            return all(
                end_ms <= frame['start_ms']
                or start_ms >= frame['start_ms'] + frame['time_ms']
                for frame in others
            )

        if is_free(desired_start):
            chosen_start = desired_start
        else:
            ordered = sorted(others, key=lambda frame: frame['start_ms'])
            moved_center = desired_start + duration / 2.0
            insert_at = sum(
                moved_center >= frame['start_ms'] + frame['time_ms'] / 2.0
                for frame in ordered
            )
            previous_end = 0
            if insert_at > 0:
                previous = ordered[insert_at - 1]
                previous_end = previous['start_ms'] + previous['time_ms']
            chosen_start = max(desired_start, previous_end)
            ordered.insert(insert_at, moved_frame)
            moved_frame['start_ms'] = chosen_start

            # 삽입 지점 뒤쪽만 밀어 기존의 다른 빈 간격은 가능한 한 유지합니다.
            current_end = chosen_start + duration
            for frame in ordered[insert_at + 1:]:
                if frame['start_ms'] < current_end:
                    frame['start_ms'] = current_end
                current_end = frame['start_ms'] + frame['time_ms']

            if current_end > 60000:
                for frame, original_start in original_positions:
                    frame['start_ms'] = original_start
                drag_original = moved_frame.get('_drag_original_start_ms')
                if drag_original is None:
                    self.motion_sequence.remove(moved_frame)
                else:
                    moved_frame['start_ms'] = drag_original
                    moved_frame.pop('_drag_original_start_ms', None)
                return False
            if current_end > self.max_seq_ms:
                self.max_seq_ms = current_end
                self.spin_max_time.setValue(current_end)

        moved_frame['start_ms'] = chosen_start
        moved_frame.pop('_drag_original_start_ms', None)
        self.motion_sequence.sort(key=lambda frame: frame['start_ms'])
        return True

    def resort_motion_sequence(self):
        if not self.motion_sequence: return
        self.motion_sequence.sort(key=lambda f: f['start_ms'])
        current_end = 0
        for f in self.motion_sequence:
            if f['start_ms'] < current_end:
                f['start_ms'] = current_end
            current_end = f['start_ms'] + f['time_ms']
            
        excess = current_end - self.max_seq_ms
        if excess > 0:
            limit_end = self.max_seq_ms
            for f in reversed(self.motion_sequence):
                if f['start_ms'] + f['time_ms'] > limit_end:
                    f['start_ms'] = limit_end - f['time_ms']
                limit_end = f['start_ms'] 
                
            if self.motion_sequence[0]['start_ms'] < 0:
                offset = 0 - self.motion_sequence[0]['start_ms']
                for f in self.motion_sequence:
                    f['start_ms'] += offset

    def apply_max_sequence_time(self):
        new_max = self.spin_max_time.value()
        total_len = sum(f['time_ms'] for f in self.motion_sequence)
        if total_len > new_max:
            QMessageBox.warning(self, "공간 부족", f"현재 블록들의 총 길이({total_len}ms)가 설정하려는 도화지 길이({new_max}ms)보다 큽니다!\n블록 길이를 줄인 후 시도하세요.")
            self.spin_max_time.setValue(self.max_seq_ms) 
            return
        self.max_seq_ms = new_max
        self.resort_motion_sequence() 
        self.refresh_timeline_ui()
        QMessageBox.information(self, "완료", f"타임라인 총 길이가 {new_max}ms로 설정되었습니다.")

    def apply_target_time(self):
        if not self.motion_sequence: return
        target_total = self.spin_target_time.value()
        current_total = max(f['start_ms'] + f['time_ms'] for f in self.motion_sequence)
        if current_total == 0: return
        
        ratio = target_total / current_total
        current_x = 0
        for f in sorted(self.motion_sequence, key=lambda x: x['start_ms']):
            new_start = int(f['start_ms'] * ratio)
            new_time = max(MIN_TIMELINE_FRAME_MS, int(f['time_ms'] * ratio))
            if new_start < current_x: new_start = current_x
            f['start_ms'] = new_start
            f['time_ms'] = new_time
            current_x = new_start + new_time
            
        if current_x > self.max_seq_ms:
            self.max_seq_ms = current_x
            self.spin_max_time.setValue(self.max_seq_ms)
            
        self.refresh_timeline_ui()
        QMessageBox.information(self, "완료", f"전체 시퀀스가 비율 스케일링 되었습니다.")

    def apply_uniform_frame_gap(self):
        if not self.motion_sequence:
            return QMessageBox.warning(self, "경고", "간격을 적용할 프레임이 없습니다.")

        gap_ms = self.spin_uniform_frame_gap.value()
        ordered = sorted(self.motion_sequence, key=lambda frame: frame['start_ms'])
        first_start = max(0, ordered[0]['start_ms'])
        planned_starts = []
        current_start = first_start
        for frame in ordered:
            planned_starts.append(current_start)
            current_start += frame['time_ms'] + gap_ms
        final_end = current_start - gap_ms

        if final_end > 60000:
            return QMessageBox.warning(
                self,
                "시간 초과",
                f"{gap_ms}ms 간격을 적용하면 종료 지점이 {final_end}ms가 되어 "
                "최대 길이 60000ms를 초과합니다.",
            )

        for frame, start_ms in zip(ordered, planned_starts):
            frame['start_ms'] = start_ms
        self.motion_sequence = ordered
        if final_end > self.max_seq_ms:
            self.max_seq_ms = final_end
            self.spin_max_time.setValue(final_end)
        self.refresh_timeline_ui()
        QMessageBox.information(
            self,
            "간격 적용 완료",
            f"모든 프레임 사이 간격을 {gap_ms}ms로 맞췄습니다.\n"
            f"첫 프레임 시작: {first_start}ms / 종료 지점: {final_end}ms",
        )

    def refresh_timeline_ui(self):
        # 로드된 예전 데이터나 일괄 시간 변경 결과도 그리기 전에 정렬하여
        # 블록이 같은 트랙에서 서로 겹치는 상태를 허용하지 않습니다.
        self.resort_motion_sequence()
        for child in self.timeline_container.findChildren(TimelineBlockWidget):
            child.setParent(None)
            child.deleteLater()
            
        for idx, frame_data in enumerate(self.motion_sequence):
            block = TimelineBlockWidget(frame_data, idx, self)
            block.setParent(self.timeline_container)
            x_pos = int(frame_data['start_ms'] * self.SCALE)
            w = int(frame_data['time_ms'] * self.SCALE) 
            block.setGeometry(x_pos, 40, w, 110)
            block.show()
            
        fixed_width = int(self.max_seq_ms * self.SCALE)
        self.timeline_container.setMinimumSize(fixed_width + 10, 165)
        if hasattr(self, 'spin_uniform_frame_gap'):
            ordered = sorted(self.motion_sequence, key=lambda frame: frame['start_ms'])
            gaps = [
                ordered[index]['start_ms']
                - (ordered[index - 1]['start_ms'] + ordered[index - 1]['time_ms'])
                for index in range(1, len(ordered))
            ]
            displayed_gap = gaps[0] if gaps and all(gap == gaps[0] for gap in gaps) else 0
            if 0 <= displayed_gap <= self.spin_uniform_frame_gap.maximum():
                self.spin_uniform_frame_gap.blockSignals(True)
                self.spin_uniform_frame_gap.setValue(displayed_gap)
                self.spin_uniform_frame_gap.blockSignals(False)
        self.refresh_timeline_meta()
        self.save_persistent_state()

    def refresh_timeline_meta(self):
        if not self.motion_sequence: 
            self.lbl_total_time.setText(f"현재 모션 종료 지점: 0ms / 타임라인 길이: {self.max_seq_ms}ms")
        else:
            current_end = max(f['start_ms'] + f['time_ms'] for f in self.motion_sequence)
            self.lbl_total_time.setText(f"현재 모션 종료 지점: {current_end}ms / 타임라인 길이: {self.max_seq_ms}ms")
        self.timeline_container.update()

    def add_to_motion(self):
        selected_items = self.frame_list_all.selectedItems()
        if not selected_items:
            return QMessageBox.warning(self, "경고", "추가할 프레임을 선택하세요.")

        current_end_ms = 0
        if self.motion_sequence: current_end_ms = max(f['start_ms'] + f['time_ms'] for f in self.motion_sequence)

        for item in selected_items:
            original_idx = item.data(Qt.UserRole)
            new_frame = copy.deepcopy(self.frames[original_idx])
            new_frame['time_ms'] = max(MIN_TIMELINE_FRAME_MS, new_frame['time_ms'])
            
            if current_end_ms + new_frame['time_ms'] > self.max_seq_ms:
                QMessageBox.warning(self, "공간 부족", f"타임라인 공간이 부족하여 추가할 수 없습니다.\n여백을 확보하거나 총 길이를 늘려주세요.")
                break 
            
            new_frame['start_ms'] = current_end_ms
            self.motion_sequence.append(new_frame)
            current_end_ms = new_frame['start_ms'] + new_frame['time_ms']
            
        self.resort_motion_sequence()
        self.refresh_timeline_ui()

    def remove_from_motion_by_idx(self, idx):
        if 0 <= idx < len(self.motion_sequence):
            self.motion_sequence.pop(idx)
            self.resort_motion_sequence()
            self.refresh_timeline_ui()

    def clear_motion(self):
        if not self.motion_sequence: return
        if QMessageBox.question(self, '확인', "모두 비우시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.motion_sequence.clear()
            self.lbl_loaded_sequence.setText("현재 타임라인: 새 시퀀스")
            self.refresh_timeline_ui()

    def save_sequence(self):
        if not self.motion_sequence: return QMessageBox.warning(self, "경고", "저장할 타임라인이 비어있습니다.")
        seq_name, ok = QInputDialog.getText(self, '저장', '새로운 시퀀스 이름 입력:')
        if ok and seq_name.strip():
            self.saved_sequences.append({
                "name": seq_name.strip(), 
                "max_seq_ms": self.max_seq_ms,
                "repeat_count": self.spin_motion_repeat.value(),
                "playback_speed": self.spin_motion_speed.value(),
                "frames": copy.deepcopy(self.motion_sequence)
            })
            self.refresh_sequence_list()
            self.save_persistent_state()
            QMessageBox.information(self, "저장 완료", f"'{seq_name.strip()}' 시퀀스 저장됨.")

    def load_saved_sequence(self, idx):
        if idx < 0 or idx >= len(self.saved_sequences):
            return
        target_seq = self.saved_sequences[idx]
        self.max_seq_ms = target_seq.get("max_seq_ms", 5000)
        self.spin_max_time.setValue(self.max_seq_ms)
        self.spin_motion_repeat.setValue(max(1, int(target_seq.get("repeat_count", 1))))
        self.spin_motion_speed.setValue(
            max(0.1, min(5.0, float(target_seq.get("playback_speed", 1.0))))
        )
        self.motion_sequence = copy.deepcopy(target_seq["frames"])
        self.refresh_timeline_ui()
        self.lbl_loaded_sequence.setText(f"현재 타임라인: {target_seq['name']} (불러옴)")
        self.tabs.setCurrentWidget(self.tab_motion)

    def load_selected_sequence_to_timeline(self):
        item = self.sequence_list_ui.currentItem()
        if item is None:
            return QMessageBox.warning(self, "경고", "불러올 저장 시퀀스를 선택하세요.")
        self.load_saved_sequence(item.data(Qt.UserRole))

    def load_sequence_from_list_item(self, item):
        idx = item.data(Qt.UserRole)
        if idx is not None:
            self.load_saved_sequence(idx)

    def open_sequence_manager(self):
        if hasattr(self, 'sequence_list_ui') and self.sequence_list_ui.currentItem():
            idx = self.sequence_list_ui.currentItem().data(Qt.UserRole)
            if idx is not None:
                self.load_saved_sequence(idx)
                return
        if not self.saved_sequences: return QMessageBox.information(self, "안내", "저장된 시퀀스가 없습니다.")
        items = [f"[{i+1}] {seq['name']} ({len(seq['frames'])} 프레임)" for i, seq in enumerate(self.saved_sequences)]
        item, ok = QInputDialog.getItem(self, "불러오기", "시퀀스 선택 (현재 덮어씌워짐):", items, 0, False)
        if ok and item:
            idx = items.index(item)
            self.load_saved_sequence(idx)

    def update_playback_buttons(self):
        self.btn_keyframe_play.setEnabled(not self.is_playing)
        self.spin_motion_repeat.setEnabled(
            not self.is_playing and not (self.is_paused and self.playback_context == "motion")
        )
        self.spin_motion_speed.setEnabled(
            not self.is_playing and not (self.is_paused and self.playback_context == "motion")
        )
        self.btn_keyframe_pause.setEnabled(self.is_playing)
        self.btn_stop_motion.setEnabled(self.is_playing or self.is_paused or self.current_timeline_ms > 0)
        if self.is_playing and self.playback_context == "motion":
            self.btn_keyframe_play.setText(
                f"▶️ 재생 중 ({self.playback_repeat_current}/{self.playback_repeat_target}, "
                f"{self.playback_speed:.1f}x)"
            )
        else:
            self.btn_keyframe_play.setText("▶️ 계속 재생" if self.is_paused else "▶️ 재생")
        if hasattr(self, 'composer_btn_play'):
            composer_active = self.playback_context == "composer"
            self.spin_composer_repeat.setEnabled(
                not self.is_playing and not (self.is_paused and composer_active)
            )
            self.spin_composer_speed.setEnabled(
                not self.is_playing and not (self.is_paused and composer_active)
            )
            self.composer_btn_play.setEnabled(not self.is_playing)
            self.composer_btn_pause.setEnabled(self.is_playing and composer_active)
            self.composer_btn_stop.setEnabled(composer_active and (self.is_playing or self.is_paused))
            if self.is_playing and composer_active:
                self.composer_btn_play.setText(
                    f"▶️ 재생 중 ({self.playback_repeat_current}/{self.playback_repeat_target}, "
                    f"{self.playback_speed:.1f}x)"
                )
            else:
                self.composer_btn_play.setText("▶️ 계속 재생" if composer_active and self.is_paused else "▶️ 재생")

    def toggle_robot_sync(self, checked):
        if checked:
            if not self.motion_sequence:
                QMessageBox.warning(self, "경고", "로봇과 동기화할 키프레임 시퀀스가 없습니다.")
                self.btn_robot_sync.blockSignals(True)
                self.btn_robot_sync.setChecked(False)
                self.btn_robot_sync.blockSignals(False)
                return
            reply = QMessageBox.question(
                self,
                "로봇 동기화",
                "⚠️ 동기화를 켜면 재생 및 타임라인 이동에 따라 로봇이 실제로 움직입니다.\n"
                "로봇 주변이 안전한지 확인하셨습니까?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes or not self.prepare_real_robot_sequence():
                self.btn_robot_sync.blockSignals(True)
                self.btn_robot_sync.setChecked(False)
                self.btn_robot_sync.blockSignals(False)
                return
            if not self.sync_sequence_start_from_robot():
                self.btn_robot_sync.blockSignals(True)
                self.btn_robot_sync.setChecked(False)
                self.btn_robot_sync.blockSignals(False)
                return
            self.robot_sync_enabled = True
            self.execute_on_real_robot = True
            self.robot_scrub_command_angles = self.joints.copy()
            self.robot_scrub_target_angles = self.joints.copy()
            self.btn_robot_sync.setText("🤖 로봇 동기화: ON")
            if hasattr(self, 'composer_btn_sync'):
                self.composer_btn_sync.blockSignals(True)
                self.composer_btn_sync.setChecked(True)
                self.composer_btn_sync.setText("🤖 로봇 동기화: ON")
                self.composer_btn_sync.blockSignals(False)
            print("[🔗 로봇 동기화 ON] 재생 및 타임라인 탐색 자세를 실제 로봇에 전송합니다.")
        else:
            self.robot_sync_enabled = False
            self.execute_on_real_robot = False
            self.robot_scrub_timer.stop()
            self.robot_scrub_target_angles = {}
            self.robot_scrub_command_angles = {}
            self.btn_robot_sync.setText("🤖 로봇 동기화: OFF")
            if hasattr(self, 'composer_btn_sync'):
                self.composer_btn_sync.blockSignals(True)
                self.composer_btn_sync.setChecked(False)
                self.composer_btn_sync.setText("🤖 로봇 동기화: OFF")
                self.composer_btn_sync.blockSignals(False)
            print("[🔓 로봇 동기화 OFF] 타임라인 조작은 3D 미리보기에만 반영됩니다.")

    def sync_sequence_start_from_robot(self):
        """첫 키프레임 보간 전에 실제 로봇 자세를 시작값으로 확정합니다."""
        target_ids = sorted(self.sequence_joint_ids() & set(self.online_joints))
        actual_angles = {}
        failed_ids = []
        for j_id in target_ids:
            angle = self.read_present_angle(j_id)
            if angle is None:
                failed_ids.append(j_id)
                continue
            actual_angles[j_id] = angle
            self.update_joint_display(j_id, angle, update_robot=False)

        if failed_ids:
            QMessageBox.warning(
                self,
                "통신 에러",
                "시퀀스 시작 자세를 읽지 못해 로봇 동기화를 중단합니다.\n"
                f"ID: {failed_ids}",
            )
            return False

        self.update_3d_robot()
        self.robot_scrub_command_angles = actual_angles.copy()
        self.robot_scrub_target_angles = actual_angles.copy()
        print(f"[✅ 시작 자세 동기화] 실제 로봇 관절 {len(actual_angles)}개의 현재 자세를 사용합니다.")
        return bool(actual_angles)

    def play_keyframe_sequence(self):
        if not self.motion_sequence:
            return QMessageBox.warning(self, "경고", "재생할 시퀀스가 없습니다.")
        end_ms = max(f['start_ms'] + f['time_ms'] for f in self.motion_sequence)
        if self.current_timeline_ms >= end_ms:
            self.current_timeline_ms = 0
        self.start_motion_playback(
            real_robot=self.robot_sync_enabled,
            start_ms=self.current_timeline_ms,
        )

    def play_motion_page_sequence(self):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        self.playback_context = "motion"
        if not self.is_paused:
            self.playback_repeat_target = self.spin_motion_repeat.value()
            self.playback_repeat_current = 1
            self.playback_speed = self.spin_motion_speed.value()
        self.play_keyframe_sequence()

    def play_motion_sequence(self, real_robot=False):
        """이전 호출부 호환용 래퍼."""
        if real_robot and not self.robot_sync_enabled:
            self.btn_robot_sync.setChecked(True)
            if not self.robot_sync_enabled:
                return
        self.play_keyframe_sequence()

    def start_motion_playback(self, real_robot=False, start_ms=0):
        resuming = self.is_paused and start_ms > 0
        if real_robot and self.robot_scrub_command_angles:
            # 수동 탐색 중 실제로 전송된 마지막 자세를 재생 시작 자세로 사용해
            # 재생 전환 순간의 점프를 줄입니다.
            for j_id, angle in self.robot_scrub_command_angles.items():
                self.joints[j_id] = angle
        self.robot_scrub_timer.stop()
        self.execute_on_real_robot = bool(real_robot and self.robot_sync_enabled)
        self.is_playing = True
        self.is_paused = False
        self.current_timeline_ms = max(0, int(start_ms))
        self.update_playback_buttons()

        self.anim_duration = max(
            f['start_ms'] + f['time_ms'] for f in self.motion_sequence
        ) / 1000.0
        # 재생 시작에는 별도의 기본/준비 자세를 사용하지 않습니다. 첫 번째
        # 타임라인 프레임을 시작 자세로 삼아 저장된 키프레임만 재생합니다.
        if not resuming:
            first_frame = min(self.motion_sequence, key=lambda f: f['start_ms'])
            self.start_angles = self.joints.copy()
            self.start_angles.update(self.normalize_angles(first_frame.get('angles', {})))
        self.anim_start_time = (
            time.time() - self.current_timeline_ms / (1000.0 * self.playback_speed)
        )
        self.anim_timer.start(16)

    def pause_motion_sequence(self):
        if not self.is_playing:
            return
        self.current_timeline_ms = max(
            0,
            int((time.time() - self.anim_start_time) * 1000 * self.playback_speed),
        )
        self.is_playing = False
        self.is_paused = True
        self.anim_timer.stop()
        self.update_playback_buttons()

    def find_default_pose_row(self):
        aliases = {"기본자세", "기본", "default", "home", "homepose"}
        for row, frame in enumerate(self.frames):
            normalized_name = "".join(str(frame.get("name", "")).lower().split())
            if normalized_name in aliases:
                return row
        return -1

    def return_to_default_pose(self):
        """사용자가 버튼을 눌렀을 때만 기본자세 프레임을 천천히 적용합니다."""
        default_row = self.find_default_pose_row()
        if default_row < 0:
            QMessageBox.warning(
                self,
                "기본자세 없음",
                "저장된 프레임에서 '기본자세'를 찾지 못했습니다.\n"
                "기본으로 사용할 프레임 이름을 '기본자세'로 변경해 주세요.",
            )
            return

        self.frame_list_ui1.setCurrentRow(default_row)
        self.apply_selected_frame()

    def anim_step(self):
        if not self.is_playing: return
        elapsed_sec = time.time() - self.anim_start_time
        t_ms = int(elapsed_sec * 1000 * self.playback_speed)
        self.current_timeline_ms = t_ms

        if self.playback_context == "composer":
            playhead_px = int(t_ms * self.COMPOSER_SCALE)
            self.composer_timeline_container.set_playhead(True, playhead_px)
            self.composer_timeline_scroll.ensureVisible(
                playhead_px, self.composer_timeline_scroll.height() // 2, 50, 0
            )
        else:
            playhead_px = int(t_ms * self.SCALE)
            self.timeline_container.set_playhead(True, playhead_px)
            self.timeline_scroll.ensureVisible(playhead_px, self.timeline_scroll.height()//2, 50, 0)
        
        if t_ms >= int(self.anim_duration * 1000):
            if self.motion_sequence:
                last_frame = max(self.motion_sequence, key=lambda f: f['start_ms'] + f['time_ms'])
                self.apply_to_real_robot(last_frame['angles'], force=True)
            if self.playback_repeat_current < self.playback_repeat_target:
                self.playback_repeat_current += 1
                self.current_timeline_ms = 0
                self.anim_start_time = time.time()
                self.start_angles.update(self.normalize_angles(last_frame.get('angles', {})))
                if self.playback_context == "composer":
                    self.composer_timeline_container.set_playhead(True, 0)
                    self.composer_timeline_scroll.horizontalScrollBar().setValue(0)
                else:
                    self.timeline_container.set_playhead(True, 0)
                    self.timeline_scroll.horizontalScrollBar().setValue(0)
                self.update_playback_buttons()
                return
            self.stop_motion_sequence()
            return

        self.scrub_timeline(t_ms)

    def stop_motion_sequence(self):
        stopped_context = self.playback_context
        self.is_playing = False
        self.is_paused = False
        self.current_timeline_ms = 0
        self.anim_timer.stop()
        self.robot_scrub_timer.stop()
        self.robot_scrub_target_angles = {}
        self.frame_apply_timer.stop()
        self.frame_apply_ids = []
        self.frame_apply_on_complete = None
        self.execute_on_real_robot = self.robot_sync_enabled
        self.timeline_container.set_playhead(True, 0)
        if hasattr(self, 'composer_timeline_container'):
            self.composer_timeline_container.set_playhead(False, 0)
        if stopped_context == "composer" and self.composer_motion_backup is not None:
            self.motion_sequence, self.max_seq_ms = self.composer_motion_backup
            self.composer_motion_backup = None
            self.playback_context = "motion"
        self.update_playback_buttons()
        for child in self.timeline_container.findChildren(TimelineBlockWidget):
            child.set_default_style()

    def scrub_timeline(self, t_ms, force_robot=False):
        self.current_timeline_ms = max(0, min(int(t_ms), self.max_seq_ms))
        if not self.is_playing:
            self.is_paused = self.current_timeline_ms > 0
            self.update_playback_buttons()
        active_frame = None
        last_completed = None
        
        for f in self.motion_sequence:
            if f['start_ms'] <= t_ms <= f['start_ms'] + f['time_ms']:
                active_frame = f
                break
            if f['start_ms'] + f['time_ms'] < t_ms:
                if not last_completed or (f['start_ms'] + f['time_ms']) > (last_completed['start_ms'] + last_completed['time_ms']):
                    last_completed = f

        idx_to_highlight = self.motion_sequence.index(active_frame) if active_frame else -1
        for child in self.timeline_container.findChildren(TimelineBlockWidget):
            if child.seq_idx == idx_to_highlight: child.set_playing_style()
            else: child.set_default_style()

        target_angles_to_render = {}
        if active_frame:
            # 일부 관절만 저장된 키프레임도 이전 자세 위에 누적합니다.
            # 누락 관절을 0도로 처리하면 시퀀스 시작 시 팔이 펴지는 등
            # 의도하지 않은 중간 자세가 발생합니다.
            base_state = self.start_angles if self.is_playing else self.joints
            prev_state = self.normalize_angles(base_state)
            for f in sorted(self.motion_sequence, key=lambda frame: frame['start_ms'] + frame['time_ms']):
                end_t = f['start_ms'] + f['time_ms']
                if end_t <= active_frame['start_ms']:
                    prev_state.update(self.normalize_angles(f['angles']))
            
            progress = (t_ms - active_frame['start_ms']) / float(active_frame['time_ms'])
            active_angles = self.normalize_angles(active_frame['angles'])
            for j_id in active_angles:
                v0 = prev_state.get(j_id, active_angles[j_id])
                v1 = active_angles.get(j_id, 0)
                target_angles_to_render[j_id] = self.interpolate_angle_shortest(v0, v1, progress)
        else:
            if last_completed: target_angles_to_render = self.normalize_angles(last_completed['angles'])
            else: target_angles_to_render = self.joints.copy()
            
        self.update_3d_robot(target_angles_to_render)
        
        if self.is_playing:
            self.apply_to_real_robot(target_angles_to_render, force=force_robot)
        elif self.robot_sync_enabled:
            self.queue_robot_scrub_target(target_angles_to_render)

    def queue_robot_scrub_target(self, angles_dict):
        """수동 타임라인 탐색 목표를 즉시 쓰지 않고 제한 속도로 추종합니다."""
        target = self.normalize_angles(angles_dict)
        if not target:
            return
        if not self.robot_scrub_command_angles:
            self.robot_scrub_command_angles = {
                j_id: self.joints.get(j_id, angle) for j_id, angle in target.items()
            }
        self.robot_scrub_target_angles = target
        self.robot_scrub_last_time = time.time()
        if not self.robot_scrub_timer.isActive():
            self.robot_scrub_timer.start()

    def step_robot_scrub(self):
        if not self.robot_sync_enabled or not self.robot_scrub_target_angles:
            self.robot_scrub_timer.stop()
            return

        now = time.time()
        elapsed = max(0.001, min(0.1, now - self.robot_scrub_last_time))
        self.robot_scrub_last_time = now
        max_step = self.robot_scrub_speed_deg_per_sec * elapsed
        next_angles = dict(self.robot_scrub_command_angles)
        reached_target = True

        for j_id, target_angle in self.robot_scrub_target_angles.items():
            current_angle = next_angles.get(j_id, self.joints.get(j_id, target_angle))
            delta = self.shortest_angle_delta(current_angle, target_angle)
            if abs(delta) > max_step:
                delta = math.copysign(max_step, delta)
                reached_target = False
            next_angles[j_id] = current_angle + delta

        if not self.write_goal_positions(
            next_angles,
            target_ids=self.robot_scrub_target_angles.keys(),
        ):
            self.robot_scrub_timer.stop()
            QMessageBox.warning(self, "통신 에러", "타임라인 자세를 로봇에 전송하지 못했습니다.")
            return

        self.robot_scrub_command_angles = next_angles
        if reached_target:
            self.robot_scrub_timer.stop()

    def normalize_angles(self, angles_dict):
        normalized = {}
        if not angles_dict:
            return normalized
        for j_id, angle in angles_dict.items():
            try:
                normalized[int(j_id)] = float(angle)
            except (TypeError, ValueError):
                print(f"[⚠️ 각도 데이터 무시] 잘못된 joint 값: {j_id}={angle}")
        return normalized

    def angle_to_ui_value(self, angle_deg):
        return max(-180, min(180, int(round(float(angle_deg)))))

    def shortest_angle_delta(self, start_deg, end_deg):
        return ((float(end_deg) - float(start_deg) + 180.0) % 360.0) - 180.0

    def interpolate_angle_shortest(self, start_deg, end_deg, progress):
        return float(start_deg) + self.shortest_angle_delta(start_deg, end_deg) * float(progress)

    def dxl_position_to_angle(self, dxl_position):
        angle_deg = (int(dxl_position) - DXL_CENTER_POSITION) * DXL_DEG_PER_STEP
        return max(DXL_MIN_DEG, min(DXL_MAX_DEG, angle_deg))

    def angle_to_dxl_position(self, angle_deg):
        angle_deg = float(angle_deg)
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        angle_deg = max(DXL_MIN_DEG, min(DXL_MAX_DEG, angle_deg))
        dxl_goal_position = int(round(DXL_CENTER_POSITION + angle_deg * DXL_STEPS_PER_DEG))
        return max(DXL_MIN_POSITION, min(DXL_MAX_POSITION, dxl_goal_position))

    def update_joint_display(self, j_id, angle_deg, update_robot=True):
        j_id = int(j_id)
        angle_deg = float(angle_deg)
        ui_angle = self.angle_to_ui_value(angle_deg)
        if j_id in self.sliders:
            self.sliders[j_id].blockSignals(True)
            self.sliders[j_id].setValue(ui_angle)
            self.sliders[j_id].blockSignals(False)
        if j_id in self.spinboxes:
            self.spinboxes[j_id].blockSignals(True)
            self.spinboxes[j_id].setValue(ui_angle)
            self.spinboxes[j_id].blockSignals(False)
        self.joints[j_id] = angle_deg
        if update_robot:
            self.update_3d_robot()

    def sequence_joint_ids(self):
        ids = set()
        for frame in self.motion_sequence:
            ids.update(self.normalize_angles(frame.get('angles', {})).keys())
        return ids

    def prepare_real_robot_sequence(self):
        if not self.port_opened:
            QMessageBox.warning(self, "통신 에러", "포트가 열려있지 않아 실제 로봇을 구동할 수 없습니다.")
            return False

        self.detect_online_joints()
        if not self.online_joints:
            QMessageBox.warning(self, "통신 에러", "온라인 모터가 없어 실제 로봇을 구동할 수 없습니다.")
            return False

        sequence_ids = self.sequence_joint_ids()
        if not sequence_ids:
            QMessageBox.warning(self, "경고", "시퀀스에 관절 각도 데이터가 없습니다.")
            return False

        missing_ids = sorted(sequence_ids - set(self.online_joints))
        if missing_ids:
            QMessageBox.warning(self, "통신 에러", f"시퀀스에 포함된 모터 중 오프라인 ID가 있습니다.\n오프라인: {missing_ids}\n모터 재탐색 후 다시 실행하세요.")
            return False

        torque_off_ids = []
        torque_read_failed_ids = []
        for j_id in sorted(sequence_ids):
            torque_state = self.read_torque_enabled(j_id)
            if torque_state is False:
                torque_off_ids.append(j_id)
            elif torque_state is None:
                torque_read_failed_ids.append(j_id)

        if torque_read_failed_ids:
            QMessageBox.warning(
                self,
                "통신 에러",
                f"토크 상태를 확인하지 못한 모터가 있어 실제 구동을 중단합니다.\nID: {torque_read_failed_ids}",
            )
            return False

        if torque_off_ids:
            print(f"   [🔒 실제 구동 준비] 토크 OFF 감지 ID {torque_off_ids} -> 전체 토크 ON 시도")
            self.set_all_torque_on()

        still_off_ids = []
        verify_failed_ids = []
        for j_id in sorted(sequence_ids):
            torque_state = self.read_torque_enabled(j_id)
            if torque_state is False:
                still_off_ids.append(j_id)
            elif torque_state is None:
                verify_failed_ids.append(j_id)

        if still_off_ids or verify_failed_ids:
            QMessageBox.warning(
                self,
                "통신 에러",
                "토크 ON 확인에 실패한 모터가 있어 실제 구동을 중단합니다.\n"
                f"OFF: {still_off_ids}\n상태 확인 실패: {verify_failed_ids}",
            )
            return False

        self.last_robot_write_time = 0
        return True

    def write_goal_positions(self, angles_dict, target_ids=None):
        if not self.port_opened:
            return False

        angles = self.normalize_angles(angles_dict)
        if not angles:
            return False

        ids = list(target_ids) if target_ids is not None else list(self.online_joints)
        self.groupSyncWrite.clearParam()
        added_count = 0
        for j_id in ids:
            j_id = int(j_id)
            if j_id not in angles:
                continue
            dxl_goal_position = self.angle_to_dxl_position(angles[j_id])
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))
            ]
            if self.groupSyncWrite.addParam(j_id, param_goal_position):
                added_count += 1
            else:
                print(f"[❌ SyncWrite 준비 실패] ID {j_id} 파라미터 추가 실패")

        if added_count == 0:
            return False

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[❌ SyncWrite 에러] 목표 위치 전송 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False

        return True

    def read_present_angle(self, j_id):
        if not hasattr(self, 'port_opened') or not self.port_opened:
            return None

        # TxOnly 쓰기의 상태 패킷이 늦게 도착하거나 읽기 응답이 순간적으로
        # 잘리는 경우가 있으므로 버퍼를 비운 뒤 최대 3회 다시 읽습니다.
        for attempt in range(3):
            if attempt > 0:
                time.sleep(0.005)
                self.portHandler.clearPort()

            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, j_id, ADDR_PRESENT_POSITION
            )
            if dxl_comm_result == COMM_SUCCESS:
                break
        else:
            print(f"[❌ Read 에러] ID {j_id} 현재 각도 읽기 3회 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return None

        if dxl_error != 0:
            # Hardware Alert가 있어도 Present Position 데이터 자체는 정상
            # 수신되므로 시작 자세 계산에는 실제 위치 값을 사용합니다.
            print(
                f"[⚠️ 모터 하드웨어 경고] ID {j_id}: "
                f"{self.packetHandler.getRxPacketError(dxl_error)} "
                "(현재 위치 값은 정상 수신)"
            )

        if j_id not in self.online_joints:
            self.online_joints.append(j_id)
            self.online_joints.sort()

        return self.dxl_position_to_angle(dxl_present_position)

    def read_torque_enabled(self, j_id):
        if not hasattr(self, 'port_opened') or not self.port_opened:
            return None

        for attempt in range(3):
            if attempt > 0:
                time.sleep(0.005)
                self.portHandler.clearPort()
            torque_value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                self.portHandler, j_id, ADDR_TORQUE_ENABLE
            )
            if dxl_comm_result == COMM_SUCCESS:
                break

        if dxl_comm_result != COMM_SUCCESS:
            print(f"[❌ Read 에러] ID {j_id} 토크 상태 3회 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return None
        if dxl_error != 0:
            # Protocol 2.0의 Alert/Hardware Error 비트는 읽기 통신 실패가
            # 아닙니다. 반환된 Torque Enable 값은 유효하므로 상태 판정에는
            # 사용하되, 실제 과부하/과열/전압 문제는 별도 경고로 남깁니다.
            print(
                f"[⚠️ 모터 하드웨어 경고] ID {j_id}: "
                f"{self.packetHandler.getRxPacketError(dxl_error)} "
                f"(토크 상태 값은 정상 수신: {'ON' if torque_value == 1 else 'OFF'})"
            )

        return torque_value == 1

    # 🚀 [원인 분석 및 해결] 23개 모터 응답 충돌(Incorrect status packet)을 막기 위해 TxOnly 사용!
    def sync_torque_group(self, group_name, joint_ids, is_on):
        print(f"👉 [그룹 토크] {group_name} 모터 전체 {'ON' if is_on else 'OFF'} 요청")

        for j_id in joint_ids:
            btn = self.torque_btns.get(j_id)
            if btn is not None and btn.isChecked() != is_on:
                # 기존 개별 토크 안전 절차(현재 위치 동기화 포함)를 그대로 사용합니다.
                btn.setChecked(is_on)

        self.update_torque_group_buttons()

    def update_torque_group_buttons(self):
        for group_name, joint_ids in self.torque_groups.items():
            btn_group = self.torque_group_btns.get(group_name)
            if btn_group is None:
                continue

            all_on = all(
                j_id in self.torque_btns and self.torque_btns[j_id].isChecked()
                for j_id in joint_ids
            )
            btn_group.blockSignals(True)
            btn_group.setChecked(all_on)
            btn_group.setText(f"{group_name} 전체 {'ON' if all_on else 'OFF'}")
            btn_group.setStyleSheet(
                ("background-color: #28a745;" if all_on else "background-color: #dc3545;")
                + " color: white; font-weight: bold; font-size: 11pt;"
            )
            btn_group.blockSignals(False)

    def sync_torque(self, j_id, is_on):
        print(f"👉 [GUI 작동 감지] {j_id}번 모터 토크 {'ON' if is_on else 'OFF'} 스위치 조작됨!")
        
        btn = self.torque_btns[j_id]

        if hasattr(self, 'port_opened') and self.port_opened:
            if is_on:
                # 직전 OFF/Goal TxOnly 명령에 대한 늦은 상태 패킷을 제거합니다.
                time.sleep(0.005)
                self.portHandler.clearPort()

                # 토크 ON 전 실제 Present Position을 먼저 읽어 GUI와 Goal Position을 맞춥니다.
                # 시작 시 online_joints 감지가 비어 있어도 여기서 직접 읽기를 시도합니다.
                angle_deg = self.read_present_angle(j_id)
                if angle_deg is None:
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                    btn.setText("OFF")
                    btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
                    self.sliders[j_id].setEnabled(False)
                    self.spinboxes[j_id].setEnabled(False)
                    QMessageBox.warning(self, "통신 에러", f"ID {j_id} 모터의 현재 각도를 읽지 못해 토크 ON을 중단했습니다.")
                    return

                self.update_joint_display(j_id, angle_deg)

                # 목표 주소(Goal Position)에 튐 방지용 현재 위치를 선등록
                dxl_goal_position = self.angle_to_dxl_position(angle_deg)
                dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, j_id, ADDR_GOAL_POSITION, dxl_goal_position)
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"[❌ TX 에러] ID {j_id} Goal 선등록 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                    btn.setText("OFF")
                    btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
                    self.sliders[j_id].setEnabled(False)
                    self.spinboxes[j_id].setEnabled(False)
                    QMessageBox.warning(self, "통신 에러", f"ID {j_id} 모터의 Goal Position 선등록에 실패해 토크 ON을 중단했습니다.")
                    return

                # Goal Position 쓰기 응답이 다음 Torque 쓰기와 섞이지 않게 비웁니다.
                time.sleep(0.005)
                self.portHandler.clearPort()

                print(f"   [🔄 동기화 완료] 개별 토크 ON 전 실제 각도 {angle_deg}도 획득 및 Goal 선매핑")

            torque_val = 1 if is_on else 0
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, j_id, ADDR_TORQUE_ENABLE, torque_val)

            # 모터의 Status Return Level 설정에 따라 TxOnly에도 응답이 올 수
            # 있으므로 다음 읽기 전에 남지 않도록 수신 버퍼를 정리합니다.
            time.sleep(0.005)
            self.portHandler.clearPort()
            
            if dxl_comm_result != COMM_SUCCESS:
                print(f"[❌ TX 에러] ID {j_id} 토크 제어 전송 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                if is_on:
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                    is_on = False
            else:
                print(f"[✅ 통신 성공] ID {j_id} 토크 {'ON' if is_on else 'OFF'} 명령 쏨 (응답 무시)")

        btn.setText("ON" if is_on else "OFF")
        btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; border-radius: 4px;" if is_on else "background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
        self.sliders[j_id].setEnabled(is_on)
        self.spinboxes[j_id].setEnabled(is_on)
        self.update_torque_group_buttons()

    # 🚀 [원인 분석 및 해결] 드래그 시 대량 패킷 충돌 방지를 위해 TxOnly 사용!
    def sync_values(self, joint_id, value, source, transmit_motor=True):
        if transmit_motor and self.frame_apply_timer.isActive():
            self.frame_apply_timer.stop()
            self.frame_apply_ids = []
            self.frame_apply_on_complete = None

        self.joints[joint_id] = value
        if source == 'slider':
            self.spinboxes[joint_id].blockSignals(True)
            self.spinboxes[joint_id].setValue(value)
            self.spinboxes[joint_id].blockSignals(False)
        elif source == 'spinbox':
            self.sliders[joint_id].blockSignals(True)
            self.sliders[joint_id].setValue(value)
            self.sliders[joint_id].blockSignals(False)
        self.update_3d_robot()
        
        if (transmit_motor and self.torque_btns[joint_id].isChecked()
                and hasattr(self, 'port_opened') and self.port_opened):
            dxl_goal_position = self.angle_to_dxl_position(value)
            
            # TxRx 대신 TxOnly 적용
            dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, joint_id, ADDR_GOAL_POSITION, dxl_goal_position)
            
            if dxl_comm_result != COMM_SUCCESS:
                print(f"[❌ TX 에러] ID {joint_id} 각도 전송 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            else:
                print(f"[✅ 통신 성공] ID {joint_id} -> {value}도 명령 쏨 (응답 무시)")

    def read_angles_from_robot(self):
        print("👉 [GUI 작동 감지] 실제 로봇 관절값 읽어오기 버튼 눌림!")
        if not self.port_opened: return QMessageBox.warning(self, "에러", "포트가 열려있지 않습니다.")
        
        # 🚀 [완벽 패치] 로봇 전원이 그새 켜졌을 수 있으므로 온라인 모터 목록을 새로 갱신합니다.
        self.detect_online_joints()
        if not self.online_joints:
             QMessageBox.warning(self, "에러", "통신 가능한 다이나믹셀 모터가 존재하지 않습니다.\n배터리 및 배선 상태를 재점검하세요.")
             return

        if QMessageBox.question(self, 'Teach Mode', f"감지된 {len(self.online_joints)}개 관절의 실제 각도를 가져오시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.No: return
        
        self.groupSyncRead.clearParam()
        for j_id in self.online_joints:
            self.groupSyncRead.addParam(j_id)
            
        # ⚠️ 값을 '읽어올' 때는 대답을 기다려야 하므로 TxRx를 그대로 둡니다. (이건 Return Level 1에서도 정상 작동함)
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[❌ Read 에러] 로봇 값 읽어오기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            QMessageBox.warning(self, "통신 에러", "물리 로봇에서 데이터를 읽어오는 데 실패했습니다.\n터미널 로그를 확인하세요.")
            return
            
        success_ids = []
        for j_id in self.online_joints:
            if self.groupSyncRead.isAvailable(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                dxl_present_position = self.groupSyncRead.getData(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                angle_deg = self.dxl_position_to_angle(dxl_present_position)
                self.update_joint_display(j_id, angle_deg, update_robot=False)
                success_ids.append(j_id)
        
        print(f"[✅ Read 성공] 다음 ID의 값을 성공적으로 읽어왔습니다: {success_ids}")        
        self.update_3d_robot()
        QMessageBox.information(self, "완료", f"실제 관절 {len(success_ids)}개의 현재 각도를 성공적으로 매핑했습니다!")

    def apply_to_real_robot(self, angles_dict, force=False):
        if not self.execute_on_real_robot or not self.port_opened: return
        now = time.time()
        if not force and (now - self.last_robot_write_time) < 0.03:
            return
        if self.write_goal_positions(angles_dict):
            self.last_robot_write_time = now

    # 🚀 [제로-저크 안전 결합 해결] 전체 토크를 켤 때, 먼저 23축 전체 물리 각도를 SyncRead로 긁어와 목표 각도(Goal Position)를 강제 일치시킵니다.
    def set_all_torque_on(self):
        print("👉 [GUI 작동 감지] 전체 토크 ON 버튼 눌림!")
        if not self.port_opened: return
        
        # 🚀 [완벽 패치] 프로그램 기동 시 로봇 전원이 꺼져 있었더라도, 토크 ON 시점에 무조건 자동 재수색하여 활성화합니다!
        if not self.online_joints:
            print("   [🔍 온라인 모터 재수색] 기동 시 감지된 관절이 없어 재탐색을 수행합니다...")
            self.detect_online_joints()
            if not self.online_joints:
                QMessageBox.warning(self, "통신 에러", "현재 연결된 다이나믹셀 모터를 스캔해내지 못했습니다.\n배터리/SMPS 전원 스위치가 단단히 켜져 있는지 확인하세요.")
                return

        # 1. 온라인이 확정된 축의 현재 실제 각도를 초고속으로 일괄 스캔하여 GUI 및 변수에 동기화
        self.groupSyncRead.clearParam()
        for j_id in self.online_joints:
            self.groupSyncRead.addParam(j_id)
            
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        read_success_ids = set()
        if dxl_comm_result == COMM_SUCCESS:
            print("   [✅ Read 성공] 일제 잠금 전 전신 관절값 스캔 성공!")
            for j_id in self.online_joints:
                if self.groupSyncRead.isAvailable(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                    dxl_present_position = self.groupSyncRead.getData(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    angle_deg = self.dxl_position_to_angle(dxl_present_position)
                    self.update_joint_display(j_id, angle_deg, update_robot=False)
                    read_success_ids.add(j_id)
        else:
            print(f"   [⚠️ 경고] 토크 ON 전 각도 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

        missing_ids = [j_id for j_id in self.online_joints if j_id not in read_success_ids]
        if missing_ids:
            print(f"   [🔁 개별 Read 재시도] SyncRead 누락 ID: {missing_ids}")
            for j_id in missing_ids:
                angle_deg = self.read_present_angle(j_id)
                if angle_deg is not None:
                    self.update_joint_display(j_id, angle_deg, update_robot=False)
                    read_success_ids.add(j_id)

        if not read_success_ids:
            QMessageBox.warning(self, "통신 에러", "토크 ON 전 현재 각도를 읽어오지 못해 전체 토크 ON을 중단했습니다.")
            return

        self.update_3d_robot()

        # 2. 동기화된 각도 데이터를 모든 Goal Position 레지스터에 선등록하여 잠금 시 snap(발작) 발생을 원천 차단
        self.groupSyncWrite.clearParam()
        for j_id in self.online_joints:
            if j_id not in read_success_ids:
                continue
            angle_deg = self.joints[j_id]
            dxl_goal_position = self.angle_to_dxl_position(angle_deg)
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))
            ]
            self.groupSyncWrite.addParam(j_id, param_goal_position)
        dxl_goal_result = self.groupSyncWrite.txPacket()
        if dxl_goal_result != COMM_SUCCESS:
            QMessageBox.warning(self, "통신 에러", f"Goal Position 선등록 실패로 전체 토크 ON을 중단했습니다.\n{self.packetHandler.getTxRxResult(dxl_goal_result)}")
            return

        # 3. 그 다음 전 관절 잠금 (Torque ON)
        self.groupSyncWriteTorque.clearParam()
        for j_id, btn in self.torque_btns.items():
            btn.blockSignals(True)
            if j_id in read_success_ids:
                btn.setEnabled(True)
                btn.setChecked(True)
                btn.setText("ON")
                btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; border-radius: 4px;")
                self.sliders[j_id].setEnabled(True)
                self.spinboxes[j_id].setEnabled(True)
                self.groupSyncWriteTorque.addParam(j_id, [1])
            else:
                # 통신 불가능 관절은 UI 비활성화 유지
                btn.setEnabled(False)
                btn.setChecked(False)
                btn.setText("OFF")
                btn.setStyleSheet("background-color: #6c757d; color: white; font-weight: bold; border-radius: 4px;")
                self.sliders[j_id].setEnabled(False)
                self.spinboxes[j_id].setEnabled(False)
            btn.blockSignals(False)
        
        if read_success_ids:
            dxl_comm_result = self.groupSyncWriteTorque.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(f"   [❌ 에러] 전체 토크 ON 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                for j_id in read_success_ids:
                    btn = self.torque_btns[j_id]
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.setText("OFF")
                    btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
                    self.sliders[j_id].setEnabled(False)
                    self.spinboxes[j_id].setEnabled(False)
                    btn.blockSignals(False)
                QMessageBox.warning(self, "통신 에러", f"전체 토크 ON 명령 전송에 실패했습니다.\n{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            else:
                print(f"   [✅ 발송 완료] 현재 각도 읽기에 성공한 {len(read_success_ids)}개 관절에 잠금 명령 쐈습니다!")

        self.update_torque_group_buttons()

    def set_all_torque_off(self):
        print("👉 [GUI 작동 감지] 전체 토크 OFF 버튼 눌림!")
        if not self.port_opened: return
        self.groupSyncWriteTorque.clearParam()
        for j_id, btn in self.torque_btns.items():
            btn.blockSignals(True)
            btn.setEnabled(j_id in self.online_joints)
            btn.setChecked(False)
            btn.setText("OFF")
            btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;" if j_id in self.online_joints else "background-color: #6c757d; color: white; font-weight: bold; border-radius: 4px;")
            self.sliders[j_id].setEnabled(False)
            self.spinboxes[j_id].setEnabled(False)
            btn.blockSignals(False)
            
            if j_id in self.online_joints:
                self.groupSyncWriteTorque.addParam(j_id, [0])
            
        if self.online_joints:
            dxl_comm_result = self.groupSyncWriteTorque.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(f"[❌ 에러] 전체 토크 OFF 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            else:
                print(f"[✅ 발송 완료] 감지된 {len(self.online_joints)}개 모터에 티칭(늘어짐) 명령 쐈습니다!")

        self.update_torque_group_buttons()

    def sync_drag_selection(self):
        if not self.is_select_mode: return
        selected_items = self.frame_list_ui1.selectedItems()
        for i in range(self.frame_list_ui1.count()):
            widget = self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(i))
            if widget:
                widget.checkbox.blockSignals(True)
                widget.checkbox.setChecked(self.frame_list_ui1.item(i) in selected_items)
                widget.checkbox.blockSignals(False)

    def execute_frame(self):
        row = self.frame_list_ui1.currentRow()
        if row < 0: return QMessageBox.warning(self, "경고", "실행할 프레임을 선택하세요!")
        frame = self.frames[row]
        self.time_spinbox.setValue(int(frame["time_ms"]))
        angles = self.normalize_angles(frame["angles"])
        for j_id, angle in angles.items():
            self.update_joint_display(j_id, angle, update_robot=False)
        for j_id, is_on in frame.get("torques", {}).items():
            j_id = int(j_id)
            if j_id in self.torque_btns: self.torque_btns[j_id].setChecked(is_on)
        self.update_3d_robot()
        if self.port_opened and any(btn.isChecked() for btn in self.torque_btns.values()):
            self.write_goal_positions(angles)
        QMessageBox.information(self, "실행 완료", f"로봇이 '{frame['name']}' 자세로 이동합니다!")

    def apply_selected_frame(self):
        row = self.frame_list_ui1.currentRow()
        if row < 0 or row >= len(self.frames):
            QMessageBox.warning(self, "경고", "불러올 프레임을 목록에서 선택하세요.")
            return

        frame = self.frames[row]
        angles = self.normalize_angles(frame.get("angles", {}))
        if not angles:
            QMessageBox.warning(self, "경고", "선택한 프레임에 적용할 각도 데이터가 없습니다.")
            return

        if self.is_playing:
            self.stop_motion_sequence()

        # 저장 시간이 짧아도 실제 로봇은 최소 1초 동안 이동시켜 급격한
        # 자세 변화를 방지합니다. 더 느리게 하려면 프레임 이동 시간을 늘립니다.
        duration_ms = max(1000, int(frame.get("time_ms", 500)))
        self.time_spinbox.setValue(int(frame.get("time_ms", 500)))

        # 저장 당시 토크 상태를 강제로 복원하지 않고, 현재 사용자가 켜 둔
        # 온라인 모터에만 저장 각도를 적용합니다.
        active_ids = [
            j_id for j_id, btn in self.torque_btns.items()
            if btn.isChecked() and j_id in self.online_joints and j_id in angles
        ]
        if self.port_opened and active_ids:
            self.frame_apply_timer.stop()
            self.frame_apply_on_complete = None

            # 목록 선택 시 GUI에는 이미 목표값이 표시될 수 있으므로, 실제 모터
            # 위치를 시작점으로 다시 읽어 갑작스러운 첫 점프를 방지합니다.
            start_angles = {}
            for j_id in active_ids:
                current_angle = self.read_present_angle(j_id)
                if current_angle is None:
                    QMessageBox.warning(self, "통신 에러", f"ID {j_id}의 현재 각도를 읽지 못해 프레임 적용을 중단했습니다.")
                    return
                start_angles[j_id] = current_angle
                self.update_joint_display(j_id, current_angle, update_robot=False)

            self.frame_apply_start_angles = start_angles
            self.frame_apply_target_angles = {j_id: angles[j_id] for j_id in active_ids}
            self.frame_apply_ids = active_ids
            self.frame_apply_duration = duration_ms / 1000.0
            self.frame_apply_start_time = time.time()
            self.frame_apply_name = frame["name"]
            self.update_3d_robot()
            self.frame_apply_timer.start()
            print(f"[🔄 프레임 이동 시작] '{frame['name']}' -> {duration_ms}ms 동안 부드럽게 이동")
        else:
            for j_id, angle in angles.items():
                self.update_joint_display(j_id, angle, update_robot=False)
            self.update_3d_robot()
            print(f"[ℹ️ 프레임 불러오기] '{frame['name']}' 각도를 GUI에 적용했습니다. 토크 ON 모터는 없습니다.")

    def step_frame_apply(self):
        if not self.frame_apply_ids:
            self.frame_apply_timer.stop()
            return

        progress = min(1.0, (time.time() - self.frame_apply_start_time) / self.frame_apply_duration)
        # 시작과 끝에서 속도가 서서히 변하는 smoothstep 보간으로 충격을 줄입니다.
        eased = progress * progress * (3.0 - 2.0 * progress)
        intermediate = {
            j_id: self.interpolate_angle_shortest(
                self.frame_apply_start_angles[j_id],
                self.frame_apply_target_angles[j_id],
                eased,
            )
            for j_id in self.frame_apply_ids
        }

        if not self.write_goal_positions(intermediate, target_ids=self.frame_apply_ids):
            self.frame_apply_timer.stop()
            self.frame_apply_ids = []
            self.frame_apply_on_complete = None
            if self.robot_sync_enabled:
                self.btn_robot_sync.setChecked(False)
            self.is_playing = False
            self.is_paused = False
            self.update_playback_buttons()
            QMessageBox.warning(self, "통신 에러", "프레임 이동 중 모터 전송에 실패했습니다.")
            return

        for j_id, angle in intermediate.items():
            self.update_joint_display(j_id, angle, update_robot=False)
        self.update_3d_robot()

        if progress >= 1.0:
            self.frame_apply_timer.stop()
            print(f"[✅ 프레임 이동 완료] '{self.frame_apply_name}' 자세 적용 완료")
            self.frame_apply_ids = []
            on_complete = self.frame_apply_on_complete
            self.frame_apply_on_complete = None
            if on_complete is not None:
                on_complete()

    def load_frame_to_ui(self, row):
        if row < 0 or row >= len(self.frames): return
        frame = self.frames[row]
        self.time_spinbox.setValue(int(frame.get("time_ms", 500)))
        for j_id, angle in self.normalize_angles(frame["angles"]).items():
            self.update_joint_display(j_id, angle, update_robot=False)
        self.update_3d_robot()

    def add_frame(self):
        duration = self.time_spinbox.value()
        frame_data = {"frame_id": uuid.uuid4().hex, "name": f"Frame {len(self.frames) + 1}", "time_ms": duration, "angles": self.joints.copy(), "torques": {j: b.isChecked() for j, b in self.torque_btns.items()}, "is_important": False}
        self.frames.append(frame_data)
        item = QListWidgetItem(self.frame_list_ui1)
        custom_widget = FrameItemWidget(frame_data, self)
        item.setSizeHint(custom_widget.sizeHint())
        self.frame_list_ui1.setItemWidget(item, custom_widget)
        self.frame_list_ui1.scrollToBottom()
        self.refresh_library_lists()

    def update_frame(self):
        row = self.frame_list_ui1.currentRow()
        if row < 0: return QMessageBox.warning(self, "경고", "재저장할 프레임을 선택하세요.")
        source_frame = self.frames[row]
        source_frame.setdefault("frame_id", uuid.uuid4().hex)
        source_frame["angles"] = self.joints.copy()
        source_frame["torques"] = {j: b.isChecked() for j, b in self.torque_btns.items()}
        source_frame["time_ms"] = self.time_spinbox.value()

        updated_sequence_frames = 0
        collections = [self.motion_sequence]
        collections.extend(seq.get("frames", []) for seq in self.saved_sequences)
        for frames in collections:
            for sequence_frame in frames:
                same_source = sequence_frame.get("frame_id") == source_frame["frame_id"]
                legacy_name_match = (
                    not sequence_frame.get("frame_id")
                    and sequence_frame.get("name") == source_frame.get("name")
                )
                if not (same_source or legacy_name_match):
                    continue
                sequence_frame["frame_id"] = source_frame["frame_id"]
                sequence_frame["name"] = source_frame["name"]
                sequence_frame["angles"] = copy.deepcopy(source_frame["angles"])
                sequence_frame["torques"] = copy.deepcopy(source_frame["torques"])
                sequence_frame["time_ms"] = source_frame["time_ms"]
                updated_sequence_frames += 1

        total_duration = sum(frame['time_ms'] for frame in self.motion_sequence)
        if total_duration > self.max_seq_ms:
            self.max_seq_ms = total_duration
            self.spin_max_time.setValue(self.max_seq_ms)
        for sequence in self.saved_sequences:
            sequence_duration = sum(frame['time_ms'] for frame in sequence.get('frames', []))
            if sequence_duration > sequence.get('max_seq_ms', 0):
                sequence['max_seq_ms'] = sequence_duration
        self.resort_motion_sequence()
        self.refresh_timeline_ui()
        widget = self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(row))
        if widget: widget.label.setText(f"[{self.frames[row]['name']}] {self.frames[row]['time_ms']}ms")
        self.refresh_library_lists()
        QMessageBox.information(
            self,
            "재저장",
            f"프레임이 재저장되었고 시퀀스 블록 {updated_sequence_frames}개에 바로 반영되었습니다.",
        )

    def rename_frame(self):
        row = self.frame_list_ui1.currentRow()
        if row < 0: return
        new_name, ok = QInputDialog.getText(self, '이름 변경', '새 프레임 이름:', text=self.frames[row]["name"])
        if ok and new_name.strip():
            self.frames[row]["name"] = new_name.strip()
            widget = self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(row))
            if widget: widget.label.setText(f"[{new_name.strip()}] {self.frames[row]['time_ms']}ms")
            self.refresh_library_lists()

    def mirror_frame(self):
        mirrored = {0: self.joints[0]*-1, 1: self.joints[1]*1, 11: self.joints[11]*-1}
        for r_id, (l_id, sign) in self.mirror_map.items():
            mirrored[l_id], mirrored[r_id] = self.joints[r_id] * sign, self.joints[l_id] * sign
        for j_id, angle in mirrored.items():
            self.update_joint_display(j_id, angle, update_robot=False)
        self.update_3d_robot()

    def toggle_select_mode(self, checked):
        self.is_select_mode = checked
        self.btn_toggle_select.setStyleSheet("background-color: #d1ecf1;" if checked else "")
        if not checked: self.frame_list_ui1.clearSelection()
        for i in range(self.frame_list_ui1.count()):
            widget = self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(i))
            if widget:
                widget.checkbox.setVisible(checked)
                if not checked: widget.checkbox.setChecked(False)

    def delete_frame(self):
        if self.is_select_mode:
            rows = [i for i in range(self.frame_list_ui1.count()) if self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(i)).checkbox.isChecked()]
            if not rows: return QMessageBox.warning(self, "경고", "삭제할 프레임을 체크해주세요.")
            if QMessageBox.question(self, '확인', "선택 삭제하시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                for idx in sorted(rows, reverse=True):
                    self.frames.pop(idx)
                    self.frame_list_ui1.takeItem(idx)
                self.refresh_library_lists()
        else:
            row = self.frame_list_ui1.currentRow()
            if row >= 0 and QMessageBox.question(self, '확인', "삭제하시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.frames.pop(row)
                self.frame_list_ui1.takeItem(row)
                self.refresh_library_lists()

    def on_all_selected(self):
        pass

    def delete_selected_saved_sequence(self):
        item = self.sequence_list_ui.currentItem()
        if item is None:
            return QMessageBox.warning(self, "경고", "삭제할 저장 시퀀스를 선택하세요.")
        self.delete_saved_sequence(item.data(Qt.UserRole))

    def delete_composer_selected_sequence(self):
        item = self.composer_source_list.currentItem()
        if item is None:
            return QMessageBox.warning(self, "경고", "삭제할 저장 시퀀스를 선택하세요.")
        self.delete_saved_sequence(item.data(Qt.UserRole))

    def delete_saved_sequence(self, sequence_idx):
        if sequence_idx is None or not (0 <= sequence_idx < len(self.saved_sequences)):
            return
        sequence_name = self.saved_sequences[sequence_idx]['name']
        reply = QMessageBox.question(
            self,
            "시퀀스 삭제",
            f"저장된 시퀀스 '{sequence_name}'을(를) 삭제하시겠습니까?\n"
            "이 작업은 되돌릴 수 없습니다.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        if self.playback_context == "composer":
            self.stop_motion_sequence()
        self.saved_sequences.pop(sequence_idx)

        remaining_entries = []
        for entry in self.sequence_composer_entries:
            entry_idx = entry['sequence_idx']
            if entry_idx == sequence_idx:
                continue
            if entry_idx > sequence_idx:
                entry['sequence_idx'] = entry_idx - 1
            remaining_entries.append(entry)
        self.sequence_composer_entries = remaining_entries
        self.pack_sequence_composer_entries()
        self.refresh_sequence_composer_timeline()
        self.refresh_sequence_list()
        self.save_persistent_state()
        QMessageBox.information(self, "삭제 완료", f"'{sequence_name}' 시퀀스를 삭제했습니다.")

    def refresh_sequence_list(self):
        if not hasattr(self, 'sequence_list_ui'):
            return
        self.sequence_list_ui.clear()
        for idx, seq in enumerate(self.saved_sequences):
            item = QListWidgetItem(
                f"[{idx+1}] {seq['name']} ({len(seq['frames'])} 프레임 / "
                f"{seq.get('repeat_count', 1)}회 / {seq.get('playback_speed', 1.0):.1f}x)"
            )
            item.setData(Qt.UserRole, idx)
            self.sequence_list_ui.addItem(item)
        self.refresh_composer_source_list()

    def refresh_library_lists(self):
        self.frame_list_all.clear()
        for idx, frame in enumerate(self.frames):
            display_text = f"{'★ ' if frame.get('is_important', False) else ''}[{frame['name']}] {frame['time_ms']}ms"
            item_all = QListWidgetItem(display_text)
            item_all.setData(Qt.UserRole, idx) 
            self.frame_list_all.addItem(item_all)
        self.refresh_sequence_list()
        self.save_persistent_state()

    def export_motion_json(self):
        if not self.motion_sequence: return
        fileName, _ = QFileDialog.getSaveFileName(self, "모션 저장", "jetson_motion_data.json", "JSON Files (*.json)")
        if fileName:
            export_data = {
                "max_seq_ms": self.max_seq_ms,
                "repeat_count": self.spin_motion_repeat.value(),
                "playback_speed": self.spin_motion_speed.value(),
                "frames": self.motion_sequence
            }
            with open(fileName, 'w', encoding='utf-8') as f:
                json.dump(export_data, f, indent=4, ensure_ascii=False)
            QMessageBox.information(self, "성공", "데이터 추출 완료!")

if __name__ == '__main__':
    if hasattr(Qt, 'AA_EnableHighDpiScaling'): QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'): QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    font = app.font()
    font.setPointSize(12) 
    app.setFont(font)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    ex = SDKMotionEditor()
    ex.show()
    sys.exit(app.exec_())
