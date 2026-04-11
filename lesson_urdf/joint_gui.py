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
JOINTS = [
    "Joint_c_right",
    "Joint_p_right",
    "Joint_r_right",
    "Joint_c_left",
    "Joint_p_left",
    "Joint_r_left",
]

LIMITS_DEG = {
    "Joint_c_right": (-90.0, 90.0),
    "Joint_p_right": (-90.0, 90.0),
    "Joint_r_right": (-90.0, 90.0),
    "Joint_c_left": (-90.0, 90.0),
    "Joint_p_left": (-90.0, 90.0),
    "Joint_r_left": (-90.0, 90.0),
}

HOME_DEG = {
    "Joint_c_right": -25.0,
    "Joint_p_right": 15.0,
    "Joint_r_right": 40.0,
    "Joint_c_left": 25.0,
    "Joint_p_left": 15.0,
    "Joint_r_left": 40.0,
}

PUBLISH_HZ = 20.0

HW_CMD_TOPIC = "leg_joints_cmd"
HW_MAP = {
    "x": "Joint_p_right",
    "y": "Joint_c_right",
    "z": "Joint_r_right",
}

PRINT_GEOM_TO_TERMINAL = True

# =========================
# RIGHT LEG GEOMETRY
# Based on your URDF
# =========================
RIGHT_C_ORIGIN = (0.0, -0.8, 0.0)
RIGHT_C_RPY = (1.5708, 0.0, 1.5708)

RIGHT_P_ORIGIN = (-0.00013659, -0.099312, 0.25)
RIGHT_P_RPY = (1.5708, 0.0, -0.0013754)

RIGHT_R_ORIGIN = (0.0, 1.6, 0.0)
RIGHT_R_RPY = (3.1416, 0.0, 0.0)

# =========================
# LEFT LEG GEOMETRY
# Based on your URDF
# =========================
LEFT_C_ORIGIN = (0.0, 0.8, 0.0)
LEFT_C_RPY = (1.5708, 0.0, 1.5708)

LEFT_P_ORIGIN = (-0.00013659, -0.099312, 0.25)
LEFT_P_RPY = (1.5708, 0.0, -0.0013754)

LEFT_R_ORIGIN = (0.0, 1.6, 0.0)
LEFT_R_RPY = (-3.1416, 0.0, 0.0)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def rotx_T(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c, -s, 0.0],
            [0.0, s,  c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def roty_T(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [ c, 0.0, s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rotz_T(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [c, -s, 0.0, 0.0],
            [s,  c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def trans_T(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def rpy_T(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rotz_T(yaw) @ roty_T(pitch) @ rotx_T(roll)


def rot_to_rpy_zyx(R: np.ndarray):
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


def fmt_vec3(v) -> str:
    return f"({float(v[0]):.4f}, {float(v[1]):.4f}, {float(v[2]):.4f})"


class JointGuiNode(Node):
    def __init__(self):
        super().__init__("lesson_urdf_joint_gui")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.hw_pub = self.create_publisher(Vector3, HW_CMD_TOPIC, 10)
        self.sim_pub = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

        self.positions_rad = {j: 0.0 for j in JOINTS}

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_joint_states)

        # Right leg fixed transforms
        self.T_right_c_fixed = trans_T(*RIGHT_C_ORIGIN) @ rpy_T(*RIGHT_C_RPY)
        self.T_right_p_fixed = trans_T(*RIGHT_P_ORIGIN) @ rpy_T(*RIGHT_P_RPY)
        self.T_right_r_fixed = trans_T(*RIGHT_R_ORIGIN) @ rpy_T(*RIGHT_R_RPY)

        # Left leg fixed transforms
        self.T_left_c_fixed = trans_T(*LEFT_C_ORIGIN) @ rpy_T(*LEFT_C_RPY)
        self.T_left_p_fixed = trans_T(*LEFT_P_ORIGIN) @ rpy_T(*LEFT_P_RPY)
        self.T_left_r_fixed = trans_T(*LEFT_R_ORIGIN) @ rpy_T(*LEFT_R_RPY)

        self.sim_joint_order = JOINTS[:]

    def set_joint_deg(self, name: str, deg: float):
        lo, hi = LIMITS_DEG[name]
        deg = clamp(deg, lo, hi)
        self.positions_rad[name] = math.radians(deg)

    def get_joint_deg(self, name: str) -> float:
        return math.degrees(self.positions_rad[name])

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS[:]
        msg.position = [self.positions_rad[j] for j in msg.name]
        self.pub.publish(msg)

    def publish_hw_command_deg(self):
        cmd = Vector3()
        cmd.x = float(self.get_joint_deg(HW_MAP["x"]))
        cmd.y = float(self.get_joint_deg(HW_MAP["y"]))
        cmd.z = float(self.get_joint_deg(HW_MAP["z"]))

        self.hw_pub.publish(cmd)
        self.get_logger().info(
            f"[ENVIADO HW] {HW_CMD_TOPIC}: x={cmd.x:.2f}°, y={cmd.y:.2f}°, z={cmd.z:.2f}°"
        )

    def publish_sim_position_commands(self):
        data = [self.positions_rad[j] for j in self.sim_joint_order]
        self.sim_pub.publish(Float64MultiArray(data=data))

    def compute_leg_fk(self, side: str):
        if side == "right":
            qc = self.positions_rad["Joint_c_right"]
            qp = self.positions_rad["Joint_p_right"]
            qr = self.positions_rad["Joint_r_right"]

            T01 = self.T_right_c_fixed @ rotz_T(qc)
            T12 = self.T_right_p_fixed @ rotz_T(qp)
            T23 = self.T_right_r_fixed @ rotz_T(qr)
        else:
            qc = self.positions_rad["Joint_c_left"]
            qp = self.positions_rad["Joint_p_left"]
            qr = self.positions_rad["Joint_r_left"]

            T01 = self.T_left_c_fixed @ rotz_T(qc)
            T12 = self.T_left_p_fixed @ rotz_T(qp)
            T23 = self.T_left_r_fixed @ rotz_T(qr)

        T02 = T01 @ T12
        T03 = T02 @ T23

        return [
            (f"T0_1_{side}", T01),
            (f"T0_2_{side}", T02),
            (f"T0_3_{side}", T03),
        ]

    def compute_fk_details(self):
        Ts = []
        Ts.extend(self.compute_leg_fk("right"))
        Ts.extend(self.compute_leg_fk("left"))

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

    def maybe_print_geom_terminal(self):
        if not PRINT_GEOM_TO_TERMINAL:
            return

        details = self.compute_fk_details()
        if not details:
            return

        T = details[2]["T"]  # right leg final matrix
        p = T[:3, 3]
        self.get_logger().info(
            f"[GEOM RIGHT LEG] xyz={fmt_vec3(p)}\n{fmt_mat4(T)}"
        )


class Window(QtWidgets.QWidget):
    def __init__(self, node: JointGuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Lesson URDF - Joint Preview + FK")

        layout = QtWidgets.QVBoxLayout()
        self.rows = {}
        self._fk_cache = []

        self.state_box = QtWidgets.QGroupBox("State details (FK / Transform matrices)")
        h = QtWidgets.QHBoxLayout()

        self.lst_mats = QtWidgets.QListWidget()
        self.lst_mats.setMaximumWidth(180)

        self.txt_mat = QtWidgets.QPlainTextEdit()
        self.txt_mat.setReadOnly(True)
        self.txt_mat.setMinimumHeight(260)

        h.addWidget(self.lst_mats)
        h.addWidget(self.txt_mat, 1)
        self.state_box.setLayout(h)

        def show_selected_matrix():
            idx = self.lst_mats.currentRow()
            if idx < 0 or idx >= len(self._fk_cache):
                return

            d = self._fk_cache[idx]
            x, y, z = d["xyz"]
            roll, pitch, yaw = d["rpy"]
            T = d["T"]

            text = (
                f"{d['frame']}:\n"
                f"xyz [m]  = ({x:.4f}, {y:.4f}, {z:.4f})\n"
                f"rpy [rad]= (roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f})\n\n"
                f"T (4x4):\n{fmt_mat4(T)}"
            )
            self.txt_mat.setPlainText(text)

        self.lst_mats.currentRowChanged.connect(lambda _: show_selected_matrix())

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

            self.node.maybe_print_geom_terminal()

        for j in JOINTS:
            row = QtWidgets.QHBoxLayout()

            label = QtWidgets.QLabel(j)
            label.setFixedWidth(110)

            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            lo, hi = LIMITS_DEG[j]
            slider.setMinimum(int(lo))
            slider.setMaximum(int(hi))
            slider.setValue(0)

            edit = QtWidgets.QLineEdit("0")
            edit.setFixedWidth(60)
            edit.setAlignment(QtCore.Qt.AlignRight)

            def apply_value_deg(val_deg: float, joint=j):
                lo2, hi2 = LIMITS_DEG[joint]
                val_deg = clamp(val_deg, lo2, hi2)
                self.node.set_joint_deg(joint, val_deg)

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

                s.blockSignals(True)
                s.setValue(int(round(val)))
                s.blockSignals(False)

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

        layout.addWidget(self.state_box)

        btns = QtWidgets.QHBoxLayout()
        btn_zero = QtWidgets.QPushButton("Zero")
        btn_home = QtWidgets.QPushButton("Home")
        btns.addWidget(btn_zero)
        btns.addWidget(btn_home)
        layout.addLayout(btns)

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
            prev_name = None
            cur_item = self.lst_mats.currentItem()
            if cur_item is not None:
                prev_name = cur_item.text()

            for joint, (s, e) in self.rows.items():
                val = float(pose_deg.get(joint, 0.0))
                lo, hi = LIMITS_DEG[joint]
                val = clamp(val, lo, hi)

                s.blockSignals(True)
                s.setValue(int(round(val)))
                s.blockSignals(False)

                e.setText(f"{val:.1f}")
                self.node.set_joint_deg(joint, val)

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

            self.node.maybe_print_geom_terminal()

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

    try:
        node.destroy_node()
    except Exception:
        pass

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()