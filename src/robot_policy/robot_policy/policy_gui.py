#!/usr/bin/env python3
"""Small Qt control panel for resetting policy inference episodes."""

import signal
import sys

import rclpy
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_srvs.srv import Trigger


class PolicyGuiNode(Node):
    """ROS client used by the policy control window."""

    def __init__(self) -> None:
        super().__init__("robot_policy_gui")
        self.reset_client = self.create_client(Trigger, "/policy/reset_episode")


class PolicyControlWindow(QWidget):
    """Minimal UI for starting a fresh inference episode."""

    def __init__(self, node: PolicyGuiNode) -> None:
        super().__init__()
        self._node = node
        self._future = None

        self.setWindowTitle("SO-101 推理控制")
        self.setMinimumWidth(360)

        self._status = QLabel("正在等待推理服务…")
        font = self._status.font()
        font.setBold(True)
        self._status.setFont(font)
        self._detail = QLabel("重置后机械臂将归零，并随机放置任务物体。")
        self._detail.setWordWrap(True)
        self._reset_button = QPushButton("重置本轮")
        self._reset_button.setEnabled(False)
        self._reset_button.clicked.connect(self._request_reset)

        layout = QVBoxLayout(self)
        layout.addWidget(self._status)
        layout.addWidget(self._detail)
        layout.addWidget(self._reset_button)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    def _tick(self) -> None:
        rclpy.spin_once(self._node, timeout_sec=0.0)
        if self._future is not None and self._future.done():
            self._finish_reset()
        elif self._future is None:
            ready = self._node.reset_client.service_is_ready()
            self._reset_button.setEnabled(ready)
            if ready and self._status.text() == "正在等待推理服务…":
                self._status.setText("推理中")

    def _request_reset(self) -> None:
        if self._future is not None or not self._reset_button.isEnabled():
            return
        self._status.setText("正在重置…")
        self._detail.setText("正在清空 ACT 动作并重置仿真。")
        self._reset_button.setEnabled(False)
        self._future = self._node.reset_client.call_async(Trigger.Request())

    def _finish_reset(self) -> None:
        future = self._future
        self._future = None
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self._status.setText("重置失败")
            self._detail.setText(str(error))
        else:
            self._status.setText("已开始新一轮" if response.success else "重置失败")
            self._detail.setText(response.message)
        self._reset_button.setEnabled(self._node.reset_client.service_is_ready())


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication([sys.argv[0]])
    node = PolicyGuiNode()
    window = PolicyControlWindow(node)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    window.show()
    try:
        app.exec_()
    finally:
        window._timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
