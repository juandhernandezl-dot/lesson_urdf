#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PyQt5 import QtWidgets, QtCore

# =========================
# CONFIG (EDITA AQUÍ)
# =========================
JOINTS = ["joint_1", "joint_2", "joint_3"]

# límites en grados (puedes cambiarlos según tu URDF)
LIMITS_DEG = {
    "joint_1": (-90.0, 90.0),
    "joint_2": (-90.0, 90.0),
    "joint_3": (-90.0, 90.0),
}

# Pose HOME (en grados) -> cambia cuando quieras
HOME_DEG = {
    "joint_1": 15.0,
    "joint_2": -25.0,
    "joint_3": 40.0,
}

PUBLISH_HZ = 20.0  # frecuencia publicación /joint_states
# =========================


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class JointGuiNode(Node):
    def __init__(self):
        super().__init__("lesson_urdf_joint_gui")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.positions_rad = {j: 0.0 for j in JOINTS}
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish)

    def set_joint_deg(self, name: str, deg: float):
        lo, hi = LIMITS_DEG[name]
        deg = clamp(deg, lo, hi)
        self.positions_rad[name] = math.radians(deg)

    def get_joint_deg(self, name: str) -> float:
        return math.degrees(self.positions_rad[name])

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS[:]  # orden fijo
        msg.position = [self.positions_rad[j] for j in msg.name]
        self.pub.publish(msg)


class Window(QtWidgets.QWidget):
    def __init__(self, node: JointGuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Lesson URDF - Joint Preview")

        layout = QtWidgets.QVBoxLayout()
        self.rows = {}

        for j in JOINTS:
            row = QtWidgets.QHBoxLayout()

            label = QtWidgets.QLabel(j)
            label.setFixedWidth(55)

            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            lo, hi = LIMITS_DEG[j]
            slider.setMinimum(int(lo))
            slider.setMaximum(int(hi))
            slider.setValue(0)

            edit = QtWidgets.QLineEdit("0")
            edit.setFixedWidth(55)
            edit.setAlignment(QtCore.Qt.AlignRight)

            def apply_value_deg(val_deg: float, joint=j):
                lo2, hi2 = LIMITS_DEG[joint]
                val_deg = clamp(val_deg, lo2, hi2)
                self.node.set_joint_deg(joint, val_deg)

            def on_slider(val, joint=j, line=edit):
                line.setText(str(val))
                apply_value_deg(float(val), joint)

            def on_edit(joint=j, s=slider, line=edit):
                try:
                    val = float(line.text())
                except ValueError:
                    # si escriben basura, vuelve al valor actual
                    line.setText(f"{self.node.get_joint_deg(joint):.1f}")
                    return
                lo2, hi2 = LIMITS_DEG[joint]
                val = clamp(val, lo2, hi2)
                s.setValue(int(round(val)))
                line.setText(f"{val:.1f}")
                apply_value_deg(val, joint)

            slider.valueChanged.connect(on_slider)
            # que funcione tanto con Enter como al perder foco
            edit.returnPressed.connect(on_edit)
            edit.editingFinished.connect(on_edit)

            row.addWidget(label)
            row.addWidget(slider)
            row.addWidget(edit)
            layout.addLayout(row)

            self.rows[j] = (slider, edit)

        # Botones
        btns = QtWidgets.QHBoxLayout()
        btn_zero = QtWidgets.QPushButton("Zero")
        btn_home = QtWidgets.QPushButton("Home")
        btns.addWidget(btn_zero)
        btns.addWidget(btn_home)
        layout.addLayout(btns)

        def set_pose_deg(pose_deg: dict):
            for joint, (s, e) in self.rows.items():
                val = float(pose_deg.get(joint, 0.0))
                lo, hi = LIMITS_DEG[joint]
                val = clamp(val, lo, hi)
                s.blockSignals(True)
                s.setValue(int(round(val)))
                s.blockSignals(False)
                e.setText(f"{val:.1f}")
                self.node.set_joint_deg(joint, val)

        def do_zero():
            set_pose_deg({j: 0.0 for j in JOINTS})

        def do_home():
            set_pose_deg(HOME_DEG)

        btn_zero.clicked.connect(do_zero)
        btn_home.clicked.connect(do_home)

        # aplica HOME al arrancar (opcional)
        do_zero()

        self.setLayout(layout)


def main():
    rclpy.init()
    node = JointGuiNode()

    app = QtWidgets.QApplication([])
    w = Window(node)
    w.show()

    # Loop Qt + spin ROS
    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)

    app.exec_()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
