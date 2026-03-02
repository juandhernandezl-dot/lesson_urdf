#!/usr/bin/env python3
import math
import numpy as np
import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from PyQt5 import QtWidgets, QtCore

# =========================
# CONFIG
# =========================
JOINTS = ["joint_p", "joint_c", "joint_r"]

HW_CMD_TOPIC = "leg_joints_cmd"
HW_MAP = {"x": "joint_p", "y": "joint_c", "z": "joint_r"}

PUBLISH_HZ = 30.0  # streaming rate

# =========================
# DH PARAMETERS
# =========================
d1 = 61.8e-3
L1 = 72.47e-3
L2 = 61.44e-3
d2 = -272.93e-3
L3 = 246e-3

TH2_CONST = math.radians(-90.0)
ALPHA2_CONST = math.radians(-90.0)

FOOT_XYZ = (0.2459344, 0.019235, -0.01007747)
FOOT_RPY = (0.0, 0.0, 0.0)


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
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )

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


class MonitorNode(Node):
    def __init__(self):
        super().__init__("moveit_joint_monitor_gui")
        self.hw_pub = self.create_publisher(Vector3, HW_CMD_TOPIC, 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self._on_js, 50)

        self.positions_rad = {j: 0.0 for j in JOINTS}

    def _on_js(self, msg: JointState):
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        for j in JOINTS:
            if j in name_to_pos:
                self.positions_rad[j] = float(name_to_pos[j])

    def get_joint_deg(self, name: str) -> float:
        return math.degrees(self.positions_rad[name])

    def publish_hw_command_deg(self):
        cmd = Vector3()
        cmd.x = float(self.get_joint_deg(HW_MAP["x"]))
        cmd.y = float(self.get_joint_deg(HW_MAP["y"]))
        cmd.z = float(self.get_joint_deg(HW_MAP["z"]))
        self.hw_pub.publish(cmd)

    def compute_fk_details(self):
        th1 = self.positions_rad["joint_p"]
        th3 = self.positions_rad["joint_c"]
        th4 = self.positions_rad["joint_r"]

        A1 = dh_T(theta=th1, d=d1, a=L1, alpha=0.0)
        A2 = dh_T(theta=TH2_CONST, d=0.0, a=0.0, alpha=ALPHA2_CONST)
        A3 = dh_T(theta=th3, d=0.0, a=L2, alpha=0.0)
        A4 = dh_T(theta=th4, d=d2, a=L3, alpha=0.0)

        T01 = A1
        T02 = T01 @ A2
        T03 = T02 @ A3
        T04 = T03 @ A4
        T0foot = T04 @ fixed_T(FOOT_XYZ, FOOT_RPY)

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
    def __init__(self, node: MonitorNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("MoveIt Monitor - Angulos + FK (solo lectura)")

        layout = QtWidgets.QVBoxLayout()

        self.tbl = QtWidgets.QTableWidget(len(JOINTS), 2)
        self.tbl.setHorizontalHeaderLabels(["Joint", "Angulo (deg)"])
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        for i, j in enumerate(JOINTS):
            self.tbl.setItem(i, 0, QtWidgets.QTableWidgetItem(j))
            self.tbl.setItem(i, 1, QtWidgets.QTableWidgetItem("0.0"))
        self.tbl.resizeColumnsToContents()
        layout.addWidget(self.tbl)

        self.state_box = QtWidgets.QGroupBox("FK / Transform matrices")
        h = QtWidgets.QHBoxLayout()
        self.lst_mats = QtWidgets.QListWidget()
        self.lst_mats.setMaximumWidth(160)
        self.txt_mat = QtWidgets.QPlainTextEdit()
        self.txt_mat.setReadOnly(True)
        h.addWidget(self.lst_mats)
        h.addWidget(self.txt_mat, 1)
        self.state_box.setLayout(h)
        layout.addWidget(self.state_box)

        # Solo CHECK (sin botón)
        self.chk_enable_hw = QtWidgets.QCheckBox("Habilitar motores (HW)  [streaming a /leg_joints_cmd]")
        self.chk_enable_hw.setChecked(False)
        layout.addWidget(self.chk_enable_hw)

        self.lbl_hw = QtWidgets.QLabel("HW: deshabilitado")
        layout.addWidget(self.lbl_hw)

        def update_hw_label():
            self.lbl_hw.setText("HW: habilitado (streaming)" if self.chk_enable_hw.isChecked() else "HW: deshabilitado")

        self.chk_enable_hw.stateChanged.connect(update_hw_label)

        self._fk_cache = []
        self.lst_mats.currentRowChanged.connect(lambda _: self._show_selected())

        self.ui_timer = QtCore.QTimer()
        self.ui_timer.timeout.connect(self.refresh)
        self.ui_timer.start(int(1000.0 / PUBLISH_HZ))

        update_hw_label()
        self.setLayout(layout)

    def refresh(self):
        for i, j in enumerate(JOINTS):
            self.tbl.item(i, 1).setText(f"{self.node.get_joint_deg(j):.2f}")

        prev = self.lst_mats.currentItem().text() if self.lst_mats.currentItem() else None
        try:
            details = self.node.compute_fk_details()
        except Exception as e:
            self.lst_mats.clear()
            self.txt_mat.setPlainText(f"[FK ERROR] {e}")
            return

        self._fk_cache = details
        self.lst_mats.blockSignals(True)
        self.lst_mats.clear()
        for d in details:
            self.lst_mats.addItem(d["frame"])
        self.lst_mats.blockSignals(False)

        idx = 0
        if prev:
            matches = self.lst_mats.findItems(prev, QtCore.Qt.MatchExactly)
            if matches:
                idx = self.lst_mats.row(matches[0])
        self.lst_mats.setCurrentRow(idx)
        self._show_selected()

        # ✅ streaming
        if self.chk_enable_hw.isChecked():
            self.node.publish_hw_command_deg()

    def _show_selected(self):
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


def main():
    rclpy.init()
    node = MonitorNode()

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