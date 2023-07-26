# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python import get_resource
from sensor_msgs.msg import Joy
import rclpy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence, QPen, QPainter, QBrush
from python_qt_binding.QtWidgets import QShortcut, QWidget, QGraphicsScene
from rclpy.qos import QoSProfile
from rqt_gui_py.plugin import Plugin

from std_msgs.msg import String


class VirtualJoy(Plugin):

    slider_factor = 1000.0

    def __init__(self, context):
        super(VirtualJoy, self).__init__(context)
        self.setObjectName('VirtualJox')

        self._node = context.node
        self._publisher = None

        self._publisher = self._node.create_publisher(
            Joy, 'joy', qos_profile=QoSProfile(depth=10))
        
        self.aBtn = False
        self.bBtn = False
        self.cBtn = False
        self.dBtn = False
        self.eBtn = False
        self.fBtn = False


        
        
        
        
        
        self._widget = QWidget()
        _, package_path = get_resource('packages', 'rqt_joy')        
        ui_file = os.path.join(
            package_path, 'share', 'rqt_joy', 'resource', 'VirtualJoy.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('VirtualJoyUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.scene = QGraphicsScene()
        self._widget.graphicsView.setScene(self.scene)
        self.pen = QPen(Qt.black, 5, Qt.SolidLine)
        self.brush = QBrush(Qt.black, Qt.SolidPattern)
        self.joy_dot_size = 10

        self._widget.x_linear_slider.valueChanged.connect(
            self._on_x_linear_slider_changed)
        self._widget.z_angular_slider.valueChanged.connect(
            self._on_z_angular_slider_changed)

        self._widget.increase_x_linear_push_button.pressed.connect(
            self._on_strong_increase_x_linear_pressed)
        self._widget.reset_x_linear_push_button.pressed.connect(
            self._on_reset_x_linear_pressed)
        self._widget.decrease_x_linear_push_button.pressed.connect(
            self._on_strong_decrease_x_linear_pressed)
        self._widget.increase_z_angular_push_button.pressed.connect(
            self._on_strong_increase_z_angular_pressed)
        self._widget.reset_z_angular_push_button.pressed.connect(
            self._on_reset_z_angular_pressed)
        self._widget.decrease_z_angular_push_button.pressed.connect(
            self._on_strong_decrease_z_angular_pressed)


        self._widget.aButton.pressed.connect(
            self._on_a_pressed)
        self._widget.bButton.pressed.connect(
            self._on_b_pressed)
        self._widget.cButton.pressed.connect(
            self._on_c_pressed)
        self._widget.dButton.pressed.connect(
            self._on_d_pressed)
        self._widget.eButton.pressed.connect(
            self._on_e_pressed)
        self._widget.fButton.pressed.connect(
            self._on_f_pressed)
        
        self.shortcut_w = QShortcut(QKeySequence(Qt.Key_W), self._widget)
        self.shortcut_w.setContext(Qt.ApplicationShortcut)
        self.shortcut_w.activated.connect(self._on_increase_x_linear_pressed)
        self.shortcut_x = QShortcut(QKeySequence(Qt.Key_X), self._widget)
        self.shortcut_x.setContext(Qt.ApplicationShortcut)
        self.shortcut_x.activated.connect(self._on_reset_x_linear_pressed)
        self.shortcut_s = QShortcut(QKeySequence(Qt.Key_S), self._widget)
        self.shortcut_s.setContext(Qt.ApplicationShortcut)
        self.shortcut_s.activated.connect(self._on_decrease_x_linear_pressed)
        self.shortcut_a = QShortcut(QKeySequence(Qt.Key_A), self._widget)
        self.shortcut_a.setContext(Qt.ApplicationShortcut)
        self.shortcut_a.activated.connect(self._on_increase_z_angular_pressed)
        self.shortcut_z = QShortcut(QKeySequence(Qt.Key_Z), self._widget)
        self.shortcut_z.setContext(Qt.ApplicationShortcut)
        self.shortcut_z.activated.connect(self._on_reset_z_angular_pressed)
        self.shortcut_d = QShortcut(QKeySequence(Qt.Key_D), self._widget)
        self.shortcut_d.setContext(Qt.ApplicationShortcut)
        self.shortcut_d.activated.connect(self._on_decrease_z_angular_pressed)

        self.shortcut_shift_w = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_W), self._widget)
        self.shortcut_shift_w.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_w.activated.connect(
            self._on_strong_increase_x_linear_pressed)
        self.shortcut_shift_x = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_X), self._widget)
        self.shortcut_shift_x.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_x.activated.connect(
            self._on_reset_x_linear_pressed)
        self.shortcut_shift_s = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_S), self._widget)
        self.shortcut_shift_s.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_s.activated.connect(
            self._on_strong_decrease_x_linear_pressed)
        self.shortcut_shift_a = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_A), self._widget)
        self.shortcut_shift_a.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_a.activated.connect(
            self._on_strong_increase_z_angular_pressed)
        self.shortcut_shift_z = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Z), self._widget)
        self.shortcut_shift_z.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_z.activated.connect(
            self._on_reset_z_angular_pressed)
        self.shortcut_shift_d = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_D), self._widget)
        self.shortcut_shift_d.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_d.activated.connect(
            self._on_strong_decrease_z_angular_pressed)

        self.shortcut_space = QShortcut(
            QKeySequence(Qt.Key_Space), self._widget)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Space), self._widget)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)

        self._widget.increase_x_linear_push_button.setToolTip(
            self._widget.increase_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] W)'))
        self._widget.reset_x_linear_push_button.setToolTip(
            self._widget.reset_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] X)'))
        self._widget.decrease_x_linear_push_button.setToolTip(
            self._widget.decrease_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] S)'))
        self._widget.increase_z_angular_push_button.setToolTip(
            self._widget.increase_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] A)'))
        self._widget.reset_z_angular_push_button.setToolTip(
            self._widget.reset_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] Z)'))
        self._widget.decrease_z_angular_push_button.setToolTip(
            self._widget.decrease_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] D)'))

        # timer to consecutively send joy messages
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)
        self.zero_joy_sent = False
           
    def _on_x_linear_slider_changed(self):
        self._widget.current_x_linear_label.setText(
            '%0.2f' % (self._widget.x_linear_slider.value() / VirtualJoy.slider_factor))
        self._on_parameter_changed()

    def _on_z_angular_slider_changed(self):
        self._widget.current_z_angular_label.setText(
            '%0.2f' % (self._widget.z_angular_slider.value() / VirtualJoy.slider_factor))
        self._on_parameter_changed()

    def _on_increase_x_linear_pressed(self):
        self._widget.x_linear_slider.setValue(
            self._widget.x_linear_slider.value() + self._widget.x_linear_slider.singleStep())

    def _on_reset_x_linear_pressed(self):
        self._widget.x_linear_slider.setValue(0)

    def _on_decrease_x_linear_pressed(self):
        self._widget.x_linear_slider.setValue(
            self._widget.x_linear_slider.value() - self._widget.x_linear_slider.singleStep())

    def _on_increase_z_angular_pressed(self):
        self._widget.z_angular_slider.setValue(
            self._widget.z_angular_slider.value() - self._widget.z_angular_slider.singleStep())

    def _on_reset_z_angular_pressed(self):
        self._widget.z_angular_slider.setValue(0)

    def _on_decrease_z_angular_pressed(self):
        self._widget.z_angular_slider.setValue(
            self._widget.z_angular_slider.value() + self._widget.z_angular_slider.singleStep())
         
    def _on_strong_increase_x_linear_pressed(self):
        self._widget.x_linear_slider.setValue(
            self._widget.x_linear_slider.value() + self._widget.x_linear_slider.pageStep())

    def _on_strong_decrease_x_linear_pressed(self):
        self._widget.x_linear_slider.setValue(
            self._widget.x_linear_slider.value() - self._widget.x_linear_slider.pageStep())

    def _on_strong_increase_z_angular_pressed(self):
        self._widget.z_angular_slider.setValue(
            self._widget.z_angular_slider.value() - self._widget.z_angular_slider.pageStep())

    def _on_strong_decrease_z_angular_pressed(self):
        self._widget.z_angular_slider.setValue(
            self._widget.z_angular_slider.value() + self._widget.z_angular_slider.pageStep())

    def _on_a_pressed(self):
        self.aBtn = not self.aBtn
        self._on_parameter_changed()
        self._widget.aButton.setFlat(not self._widget.aButton.isFlat())

    def _on_b_pressed(self):
        self.bBtn = not self.bBtn
        self._on_parameter_changed()
        self._widget.bButton.setFlat(not self._widget.bButton.isFlat())

    def _on_c_pressed(self):
        self.cBtn = not self.cBtn
        self._on_parameter_changed()
        self._widget.cButton.setFlat(not self._widget.cButton.isFlat())

    def _on_d_pressed(self):
        self.dBtn = not self.dBtn
        self._on_parameter_changed()
        self._widget.dButton.setFlat(not self._widget.dButton.isFlat())

    def _on_e_pressed(self):
        self.eBtn = not self.eBtn
        self._on_parameter_changed()
        self._widget.eButton.setFlat(not self._widget.eButton.isFlat())

    def _on_f_pressed(self):
        self.fBtn = not self.fBtn
        self._on_parameter_changed()
        self._widget.fButton.setFlat(not self._widget.fButton.isFlat())

    def _on_parameter_changed(self):
        self._send_joy(
            self._widget.x_linear_slider.value() / VirtualJoy.slider_factor,
            self._widget.z_angular_slider.value() / VirtualJoy.slider_factor,
            self.aBtn, self.bBtn, self.cBtn, self.dBtn, self.eBtn, self.fBtn)
        self._draw_joypad_dot(
            self._widget.x_linear_slider.value() / VirtualJoy.slider_factor,
            self._widget.z_angular_slider.value() / VirtualJoy.slider_factor)
        
    def _send_joy(self, x_linear, z_angular, a_value, b_value, c_value, d_value, e_value, f_value):
        if self._publisher is None:
            return
        joypad = Joy()
        joypad.axes = [0.0] * 2
        joypad.buttons = [0] * 6
        joypad.axes[0] = x_linear
        joypad.axes[1] = z_angular
        joypad.buttons[0] = int(a_value)
        joypad.buttons[1] = int(b_value)
        joypad.buttons[2] = int(c_value)
        joypad.buttons[3] = int(d_value)
        joypad.buttons[4] = int(e_value)
        joypad.buttons[5] = int(f_value)
        self._publisher.publish(joypad)

    def _draw_joypad_dot(self, x_linear, z_angular):
        self.scene.clear()
        graphichs_size = self._widget.graphicsView.viewport().size()
        graphicsView_height = (graphichs_size.width() - 2*self.joy_dot_size) / 2
        graphicsView_width = (graphichs_size.height() - 2*self.joy_dot_size) / 2
        dot_x_pos = -x_linear * graphicsView_width
        dot_y_pos = z_angular * graphicsView_height
        self.scene.addEllipse(
            dot_y_pos, dot_x_pos,
            self.joy_dot_size, self.joy_dot_size,
            self.pen, self.brush)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._update_parameter_timer.stop()
        self._unregister_publisher()
