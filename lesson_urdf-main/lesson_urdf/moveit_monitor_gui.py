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

PUBLISH_HZ = 30.0  # tasa refresco monitor

PRINT_GEOM_TO_TERMINAL = False

# =========================
# PARÁMETROS GEOMÉTRICOS REALES
# derivados del CAD / URDF
# =========================

# base(c) -> p  (en el frame de cadera)
D_CP = (-0.018425, -7.5056e-05, 0.0728)

# p -> r
D_PR = (0.18787, 0.0, 0.064085)

# r -> foot
D_RF = (0.16093478, 0.00001944, 0.03845750)

# Rotación fija entre frame c y frame p
# viene de joint_p origin rpy="1.5708 0 -1.5708"
RPY_CP = (1.5708, 0.0, -1.5708)


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
            [c, 0.0, s, 0.0],
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
    """
    Convención ZYX:
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    return rotz_T(yaw) @ roty_T(pitch) @ rotx_T(roll)


def rot_to_rpy_zyx(R: np.ndarray):
    """
    Devuelve (yaw, pitch, roll) en convención ZYX.
    """
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


class MonitorNode(Node):
    def __init__(self):
        super().__init__("moveit_joint_monitor_gui")

        self.hw_pub = self.create_publisher(Vector3, HW_CMD_TOPIC, 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self._on_js, 50)

        self.positions_rad = {j: 0.0 for j in JOINTS}

        # MTH fija c -> p (offset + reorientación fija)
        self.T_cp_fixed = trans_T(*D_CP) @ rpy_T(*RPY_CP)

        # MTH fijas p -> r y r -> foot (solo traslación)
        self.T_pr_fixed = trans_T(*D_PR)
        self.T_rf_fixed = trans_T(*D_RF)

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
        """
        Método geométrico expresado con matrices homogéneas.

        Convención:
        - joint_c: giro alrededor de Z de base
        - joint_p: giro alrededor de Z local con signo negativo
        - joint_r: giro alrededor de Z local con signo negativo

        La articulación fantasma queda absorbida en:
            T_cp_fixed = Trans(D_CP) * RPY_CP
        """
        qc = self.positions_rad["joint_c"]
        qp = self.positions_rad["joint_p"]
        qr = self.positions_rad["joint_r"]

        # 0 -> c
        T01 = rotz_T(qc)

        # c -> p
        T12 = self.T_cp_fixed @ rotz_T(-qp)

        # p -> r
        T23 = self.T_pr_fixed @ rotz_T(-qr)

        # r -> foot
        T34 = self.T_rf_fixed

        # acumuladas
        T02 = T01 @ T12
        T03 = T02 @ T23
        T04 = T03 @ T34

        Ts = [
            ("T0_1", T01),      # base -> c
            ("T0_2", T02),      # base -> p
            ("T0_3", T03),      # base -> r
            ("T0_4", T04),      # base -> foot
            ("T0_foot", T04),   # alias
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

    def maybe_print_geom_terminal(self):
        if not PRINT_GEOM_TO_TERMINAL:
            return
        details = self.compute_fk_details()
        T = details[-1]["T"]
        p = T[:3, 3]
        self.get_logger().info(
            f"[GEOM T0_foot] xyz={fmt_vec3(p)}\n{fmt_mat4(T)}"
        )


class Window(QtWidgets.QWidget):
    def __init__(self, node: MonitorNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("MoveIt Monitor - Ángulos + FK geométrica")

        layout = QtWidgets.QVBoxLayout()

        # =========================
        # Tabla de joints
        # =========================
        self.tbl = QtWidgets.QTableWidget(len(JOINTS), 2)
        self.tbl.setHorizontalHeaderLabels(["Joint", "Ángulo (deg)"])
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)

        for i, j in enumerate(JOINTS):
            self.tbl.setItem(i, 0, QtWidgets.QTableWidgetItem(j))
            self.tbl.setItem(i, 1, QtWidgets.QTableWidgetItem("0.0"))

        self.tbl.resizeColumnsToContents()
        layout.addWidget(self.tbl)

        # =========================
        # Panel de matrices
        # =========================
        self.state_box = QtWidgets.QGroupBox("FK geométrica / Transform matrices")
        h = QtWidgets.QHBoxLayout()

        self.lst_mats = QtWidgets.QListWidget()
        self.lst_mats.setMaximumWidth(160)

        self.txt_mat = QtWidgets.QPlainTextEdit()
        self.txt_mat.setReadOnly(True)

        h.addWidget(self.lst_mats)
        h.addWidget(self.txt_mat, 1)
        self.state_box.setLayout(h)
        layout.addWidget(self.state_box)

        # =========================
        # Streaming a HW
        # =========================
        self.chk_enable_hw = QtWidgets.QCheckBox(
            "Habilitar motores (HW)  [streaming a /leg_joints_cmd]"
        )
        self.chk_enable_hw.setChecked(False)
        layout.addWidget(self.chk_enable_hw)

        self.lbl_hw = QtWidgets.QLabel("HW: deshabilitado")
        layout.addWidget(self.lbl_hw)

        def update_hw_label():
            self.lbl_hw.setText(
                "HW: habilitado (streaming)"
                if self.chk_enable_hw.isChecked()
                else "HW: deshabilitado"
            )

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

        if self.chk_enable_hw.isChecked():
            self.node.publish_hw_command_deg()

        self.node.maybe_print_geom_terminal()

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