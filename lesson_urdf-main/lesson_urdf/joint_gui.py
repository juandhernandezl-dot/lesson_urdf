#!/usr/bin/env python3
import math
import numpy as np
import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from PyQt5 import QtWidgets, QtCore
from std_msgs.msg import Float64MultiArray

# =========================
# CONFIG
# =========================
JOINTS = ["joint_p", "joint_c", "joint_r"]

LIMITS_DEG = {
    "joint_p": (-90.0, 90.0),
    "joint_c": (-90.0, 90.0),
    "joint_r": (-90.0, 90.0),
}

HOME_DEG = {
    "joint_p": 15.0,
    "joint_c": -25.0,
    "joint_r": 40.0,
}

PUBLISH_HZ = 20.0

HW_CMD_TOPIC = "leg_joints_cmd"
HW_MAP = {"x": "joint_p", "y": "joint_c", "z": "joint_r"}

# =========================
# DH PARAMETERS
# =========================
d1 = 61.8e-3
L1 = 72.47e-3
L2 = 61.44e-3
d2 = -272.93e-3
L3 = 246e-3

# fila 2 "fantasma": theta=-90°, d=0, a=0, alpha=-90°
TH2_CONST = math.radians(-90.0)
ALPHA2_CONST = math.radians(-90.0)

# =========================
# FOOT LINK (del URDF joint_foot): link_r -> foot_link
# =========================
FOOT_XYZ = (0.2459344, 0.019235, -0.01007747)  # [m]
FOOT_RPY = (0.0, 0.0, 0.0)           # [rad]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def dh_T(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rpy_T(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) * Ry(pitch) * Rx(roll)"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,      cp * sr,               cp * cr],
    ], dtype=float)

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    return T


def fixed_T(xyz, rpy) -> np.ndarray:
    T = rpy_T(rpy[0], rpy[1], rpy[2])
    T[0, 3] = float(xyz[0])
    T[1, 3] = float(xyz[1])
    T[2, 3] = float(xyz[2])
    return T


def rot_to_rpy_zyx(R: np.ndarray):
    """Devuelve (yaw, pitch, roll) para convención ZYX."""
    r11, r21, r31 = R[0, 0], R[1, 0], R[2, 0]
    r32, r33 = R[2, 1], R[2, 2]
    pitch = math.atan2(-r31, math.sqrt(r11 * r11 + r21 * r21))

    if abs(math.cos(pitch)) < 1e-9:
        yaw = math.atan2(-R[1, 2], R[1, 1])
        roll = 0.0
    else:
        yaw = math.atan2(r21, r11)
        roll = math.atan2(r32, r33)

    return yaw, pitch, roll


def fmt_mat4(T: np.ndarray) -> str:
    return "\n".join(" ".join(f"{v: .4f}" for v in row) for row in T)


class JointGuiNode(Node):
    def __init__(self):
        super().__init__("lesson_urdf_joint_gui")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.hw_pub = self.create_publisher(Vector3, HW_CMD_TOPIC, 10)
        self.sim_pub = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)
        self.sim_joint_order = ["joint_p", "joint_c", "joint_r"]

        self.positions_rad = {j: 0.0 for j in JOINTS}
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_joint_states)

    def set_joint_deg(self, name: str, deg: float):
        lo, hi = LIMITS_DEG[name]
        deg = clamp(deg, lo, hi)
        self.positions_rad[name] = math.radians(deg)

    def get_joint_deg(self, name: str) -> float:
        return math.degrees(self.positions_rad[name])

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS[:]  # orden fijo
        msg.position = [self.positions_rad[j] for j in msg.name]
        self.pub.publish(msg)

    def publish_hw_command_deg(self):
        x_joint = HW_MAP["x"]
        y_joint = HW_MAP["y"]
        z_joint = HW_MAP["z"]

        cmd = Vector3()
        cmd.x = float(self.get_joint_deg(x_joint))
        cmd.y = float(self.get_joint_deg(y_joint))
        cmd.z = float(self.get_joint_deg(z_joint))

        self.hw_pub.publish(cmd)
        self.get_logger().info(
            f"[ENVIADO HW] {HW_CMD_TOPIC}: x={cmd.x:.2f}°, y={cmd.y:.2f}°, z={cmd.z:.2f}°"
        )

    def publish_sim_position_commands(self):
    # Convert deg -> rad in the controller joint order
        data = []
        for j in self.sim_joint_order:
            deg = self.get_joint_deg(j)
            data.append(math.radians(deg))
        self.sim_pub.publish(Float64MultiArray(data=data))

    def compute_fk_details(self):
        """
        FK ACUMULADO DESDE BASE:
        DH: 4 filas (incluye fila 2 fija "fantasma") + fijo (joint_foot del URDF)

        Muestra 5 matrices:
          - T0_1
          - T0_2 (phantom)
          - T0_3
          - T0_4
          - T0_foot (hasta foot_link)
        """
        th1 = self.positions_rad["joint_p"]
        th3 = self.positions_rad["joint_c"]
        th4 = self.positions_rad["joint_r"]

        # --- DH según tu tabla ---
        A1 = dh_T(theta=th1, d=d1, a=L1, alpha=0.0)
        A2 = dh_T(theta=TH2_CONST, d=0.0, a=0.0, alpha=ALPHA2_CONST)  # fantasma
        A3 = dh_T(theta=th3, d=0.0, a=L2, alpha=0.0)
        A4 = dh_T(theta=th4, d=d2, a=L3, alpha=0.0)

        T01 = A1
        T02 = T01 @ A2
        T03 = T02 @ A3
        T04 = T03 @ A4

        # --- fijo del URDF: link_r -> foot_link ---
        T4_foot = fixed_T(FOOT_XYZ, FOOT_RPY)
        T0foot = T04 @ T4_foot

        Ts = [
            ("T0_1", T01),
            ("T0_2 (phantom)", T02),
            ("T0_3", T03),
            ("T0_4", T04),
            ("T0_foot", T0foot),
        ]

        details = []
        for name, T in Ts:
            p = T[:3, 3]
            R = T[:3, :3]
            yaw, pitch, roll = rot_to_rpy_zyx(R)
            details.append(
                {
                    "frame": name,
                    "xyz": (float(p[0]), float(p[1]), float(p[2])),
                    "rpy": (float(roll), float(pitch), float(yaw)),
                    "T": T,
                }
            )
        return details


class Window(QtWidgets.QWidget):
    def __init__(self, node: JointGuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Lesson URDF - Joint Preview + FK")

        layout = QtWidgets.QVBoxLayout()
        self.rows = {}

        # =========================
        # Sliders
        # =========================
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

            def refresh_state_details():
                prev_name = None
                cur_item = self.lst_mats.currentItem()
                if cur_item is not None:
                    prev_name = cur_item.text() 
                try:
                    details = self.node.compute_fk_details()
                except Exception as e:
                    self._fk_cache = []
                    self.lst_mats.clear()
                    self.txt_mat.setPlainText(f"[FK ERROR] {e}")
                    return

                self._fk_cache = details

                self.lst_mats.blockSignals(True)
                self.lst_mats.clear()
                for d in details:
                    self.lst_mats.addItem(d["frame"])
                self.lst_mats.blockSignals(False)

                restore_row = 0
                if prev_name is not None:
                    matches = self.lst_mats.findItems(prev_name, QtCore.Qt.MatchExactly)
                    if matches:
                        restore_row = self.lst_mats.row(matches[0])

                if self.lst_mats.count() > 0:
                    self.lst_mats.setCurrentRow(restore_row)
                    show_selected_matrix()

            def on_slider(val, joint=j, line=edit):
                line.setText(str(val))
                apply_value_deg(float(val), joint)
                self.node.publish_sim_position_commands()
                refresh_state_details()

            def on_edit(joint=j, s=slider, line=edit):
                try:
                    val = float(line.text())
                except ValueError:
                    line.setText(f"{self.node.get_joint_deg(joint):.1f}")
                    return
                lo2, hi2 = LIMITS_DEG[joint]
                val = clamp(val, lo2, hi2)
                s.setValue(int(round(val)))
                line.setText(f"{val:.1f}")
                apply_value_deg(val, joint)
                self.node.publish_sim_position_commands()
                refresh_state_details()

            slider.valueChanged.connect(on_slider)
            edit.returnPressed.connect(on_edit)
            edit.editingFinished.connect(on_edit)

            row.addWidget(label)
            row.addWidget(slider)
            row.addWidget(edit)
            layout.addLayout(row)
            self.rows[j] = (slider, edit)

        # =========================
        # State details (lista clickeable)
        # =========================
        self.state_box = QtWidgets.QGroupBox("State details (FK / Transform matrices)")
        h = QtWidgets.QHBoxLayout()

        self.lst_mats = QtWidgets.QListWidget()
        self.lst_mats.setMaximumWidth(140)

        self.txt_mat = QtWidgets.QPlainTextEdit()
        self.txt_mat.setReadOnly(True)
        self.txt_mat.setMinimumHeight(260)

        h.addWidget(self.lst_mats)
        h.addWidget(self.txt_mat, 1)
        self.state_box.setLayout(h)
        layout.addWidget(self.state_box)

        self._fk_cache = []

        def show_selected_matrix():
            idx = self.lst_mats.currentRow()
            if idx < 0 or idx >= len(self._fk_cache):
                return
            d = self._fk_cache[idx]
            x, y, z = d["xyz"]
            roll, pitch, yaw = d["rpy"]
            T = d["T"]

            self.txt_mat.setPlainText(
                f"{d['frame']}:\n"
                f"xyz [m]  = ({x:.4f}, {y:.4f}, {z:.4f})\n"
                f"rpy [rad]= (roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f})\n\n"
                f"T (4x4):\n{fmt_mat4(T)}"
            )

        self.lst_mats.currentRowChanged.connect(lambda _: show_selected_matrix())

        # =========================
        # Botones básicos
        # =========================
        btns = QtWidgets.QHBoxLayout()
        btn_zero = QtWidgets.QPushButton("Zero")
        btn_home = QtWidgets.QPushButton("Home")
        btns.addWidget(btn_zero)
        btns.addWidget(btn_home)
        layout.addLayout(btns)

        # =========================
        # Hardware
        # =========================
        hw_row = QtWidgets.QHBoxLayout()
        self.chk_enable_hw = QtWidgets.QCheckBox("Habilitar motores")
        self.chk_enable_hw.setChecked(False)

        btn_send = QtWidgets.QPushButton("Enviar")
        btn_send.setToolTip(f"Publica 1 comando a {HW_CMD_TOPIC}")

        hw_row.addWidget(self.chk_enable_hw)
        hw_row.addStretch(1)
        hw_row.addWidget(btn_send)
        layout.addLayout(hw_row)

        self.lbl_hw_status = QtWidgets.QLabel("HW: deshabilitado")
        layout.addWidget(self.lbl_hw_status)

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

            # refresca FK
            details = self.node.compute_fk_details()
            self._fk_cache = details
            self.lst_mats.blockSignals(True)
            self.lst_mats.clear()
            for d in details:
                self.lst_mats.addItem(d["frame"])
            self.lst_mats.blockSignals(False)
            if self.lst_mats.count() > 0:
                self.lst_mats.setCurrentRow(0)
                # pinta la primera
                d = self._fk_cache[0]
                x, y, z = d["xyz"]
                roll, pitch, yaw = d["rpy"]
                T = d["T"]
                self.txt_mat.setPlainText(
                    f"{d['frame']}:\n"
                    f"xyz [m]  = ({x:.4f}, {y:.4f}, {z:.4f})\n"
                    f"rpy [rad]= (roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f})\n\n"
                    f"T (4x4):\n{fmt_mat4(T)}"
                )

        def do_zero():
            set_pose_deg({j: 0.0 for j in JOINTS})
            self.node.publish_sim_position_commands()

        def do_home():
            set_pose_deg(HOME_DEG)
            self.node.publish_sim_position_commands()

        def update_hw_label():
            self.lbl_hw_status.setText(
                "HW: habilitado" if self.chk_enable_hw.isChecked() else "HW: deshabilitado"
            )

        def do_send():
            if not self.chk_enable_hw.isChecked():
                QtWidgets.QMessageBox.warning(
                    self,
                    "Bloqueado",
                    "Activa primero 'Habilitar motores' para poder enviar comandos al robot real.",
                )
                return

            xj, yj, zj = HW_MAP["x"], HW_MAP["y"], HW_MAP["z"]
            msg = (
                "¿Enviar comando al robot real?\n\n"
                f"x ({xj}) = {self.node.get_joint_deg(xj):.2f}°\n"
                f"y ({yj}) = {self.node.get_joint_deg(yj):.2f}°\n"
                f"z ({zj}) = {self.node.get_joint_deg(zj):.2f}°"
            )
            ret = QtWidgets.QMessageBox.question(
                self,
                "Confirmar envío",
                msg,
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if ret != QtWidgets.QMessageBox.Yes:
                return

            self.node.publish_hw_command_deg()

        btn_zero.clicked.connect(do_zero)
        btn_home.clicked.connect(do_home)
        self.chk_enable_hw.stateChanged.connect(update_hw_label)
        btn_send.clicked.connect(do_send)

        # init
        do_zero()
        update_hw_label()

        self.setLayout(layout)


def main():
    rclpy.init()
    node = JointGuiNode()

    app = QtWidgets.QApplication([])
    w = Window(node)
    w.show()

    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)

    signal.signal(signal.SIGINT, lambda *args: app.quit())
    signal.signal(signal.SIGTERM, lambda *args: app.quit())

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()