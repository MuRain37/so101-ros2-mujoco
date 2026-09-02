#!/usr/bin/env python3
"""Small Qt control panel for the dataset recorder services."""

import signal
import sys
import time

import rclpy
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QShortcut,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_srvs.srv import Trigger


class DatasetGuiNode(Node):
    """ROS clients used by the Qt control panel."""

    def __init__(self) -> None:
        super().__init__("so101_dataset_gui")
        self.start_client = self.create_client(Trigger, "/dataset/start_episode")
        self.stop_client = self.create_client(Trigger, "/dataset/stop_episode")
        self.cancel_client = self.create_client(Trigger, "/dataset/cancel_episode")

    def bag_is_running(self) -> bool:
        return any(
            name == "rosbag2_recorder"
            for name, _namespace in self.get_node_names_and_namespaces()
        )


class DatasetControlWindow(QWidget):
    """Minimal start/stop UI that keeps all ROS calls asynchronous."""

    def __init__(self, node: DatasetGuiNode) -> None:
        super().__init__()
        self._node = node
        self._future = None
        self._action = None
        self._bag_running = False
        self._close_after_stop = False
        self._allow_close = False
        self._graph_ticks = 0
        self._recording_started_at = None
        self._elapsed_seconds = 0.0

        self.setWindowTitle("SO-101 数据集录制")
        self.setMinimumWidth(440)

        self._status = QLabel("正在等待录制服务…")
        font = self._status.font()
        font.setBold(True)
        self._status.setFont(font)
        self._detail = QLabel("可以先调整机械臂，再开始录制。")
        self._detail.setWordWrap(True)
        self._elapsed = QLabel("录制时长：00:00:00")

        self._start_button = QPushButton("开始录制 (R)")
        self._stop_button = QPushButton("停止录制 (S)")
        self._cancel_button = QPushButton("取消录制 (Space)")
        self._start_button.clicked.connect(self._start_recording)
        self._stop_button.clicked.connect(self._stop_recording)
        self._cancel_button.clicked.connect(self._cancel_recording)
        self._start_button.setEnabled(False)
        self._stop_button.setEnabled(False)
        self._cancel_button.setEnabled(False)
        self._start_shortcut = QShortcut(QKeySequence("R"), self)
        self._stop_shortcut = QShortcut(QKeySequence("S"), self)
        self._cancel_shortcut = QShortcut(QKeySequence("Space"), self)
        self._start_shortcut.activated.connect(self._start_from_shortcut)
        self._stop_shortcut.activated.connect(self._stop_recording)
        self._cancel_shortcut.activated.connect(self._cancel_recording)

        buttons = QHBoxLayout()
        buttons.addWidget(self._start_button)
        buttons.addWidget(self._stop_button)
        buttons.addWidget(self._cancel_button)
        layout = QVBoxLayout(self)
        layout.addWidget(self._status)
        layout.addWidget(self._detail)
        layout.addWidget(self._elapsed)
        layout.addLayout(buttons)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    def _set_state(self, status: str, detail: str | None = None) -> None:
        self._status.setText(status)
        if detail is not None:
            self._detail.setText(detail)

    def _tick(self) -> None:
        rclpy.spin_once(self._node, timeout_sec=0.0)
        self._update_elapsed()
        if self._future is not None and self._future.done():
            self._finish_request()

        self._graph_ticks += 1
        if self._graph_ticks >= 10:
            self._graph_ticks = 0
            self._refresh_availability()

    def _refresh_availability(self) -> None:
        ready = (
            self._node.start_client.service_is_ready()
            and self._node.stop_client.service_is_ready()
            and self._node.cancel_client.service_is_ready()
        )
        if not ready:
            if self._future is None:
                self._set_state("等待录制服务", "录制服务尚未就绪。")
            self._start_button.setEnabled(False)
            self._stop_button.setEnabled(False)
            self._cancel_button.setEnabled(False)
            return

        bag_running = self._node.bag_is_running()
        if bag_running and not self._bag_running and self._recording_started_at is None:
            self._start_elapsed()
        elif not bag_running and self._bag_running and self._future is None:
            self._stop_elapsed()
        self._bag_running = bag_running
        if self._future is not None:
            self._start_button.setEnabled(False)
            self._stop_button.setEnabled(False)
            self._cancel_button.setEnabled(False)
            return

        self._start_button.setEnabled(not self._bag_running)
        self._stop_button.setEnabled(self._bag_running)
        self._cancel_button.setEnabled(self._bag_running)
        if self._bag_running and self._status.text() not in ("录制中", "停止失败"):
            self._set_state("录制中", "检测到数据集 rosbag 正在录制。")
        elif not self._bag_running and self._status.text() in (
            "正在等待录制服务…",
            "等待录制服务",
        ):
            self._set_state("空闲", "可以先调整机械臂，再开始录制。")

    def _start_from_shortcut(self) -> None:
        self._start_recording()

    def _start_recording(self) -> None:
        if self._future is not None or not self._start_button.isEnabled():
            return
        self._elapsed_seconds = 0.0
        self._recording_started_at = None
        self._update_elapsed()
        self._action = "start"
        self._set_state("正在启动录制…", "正在检查话题并启动 rosbag。")
        self._start_button.setEnabled(False)
        self._stop_button.setEnabled(False)
        self._cancel_button.setEnabled(False)
        self._future = self._node.start_client.call_async(Trigger.Request())

    def _stop_recording(self) -> None:
        if self._future is not None or not self._stop_button.isEnabled():
            return
        self._action = "stop"
        self._set_state("正在停止录制…", "正在安全写盘并验证数据，请稍候。")
        self._start_button.setEnabled(False)
        self._stop_button.setEnabled(False)
        self._cancel_button.setEnabled(False)
        self._future = self._node.stop_client.call_async(Trigger.Request())

    def _cancel_recording(self) -> None:
        if self._future is not None or not self._cancel_button.isEnabled():
            return
        self._action = "cancel"
        self._set_state("正在取消录制…", "正在删除本次未完成的数据。")
        self._start_button.setEnabled(False)
        self._stop_button.setEnabled(False)
        self._cancel_button.setEnabled(False)
        self._future = self._node.cancel_client.call_async(Trigger.Request())

    def _finish_request(self) -> None:
        future = self._future
        action = self._action
        self._future = None
        self._action = None
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self._set_state("操作失败", str(error))
            self._close_after_stop = False
            self._refresh_availability()
            return


        if action == "start":
            self._bag_running = response.success
            if response.success:
                self._start_elapsed()
            self._set_state("录制中" if response.success else "启动失败", response.message)
        else:
            self._bag_running = not response.success and self._node.bag_is_running()
            if not self._bag_running:
                self._stop_elapsed()
            if action == "cancel":
                self._set_state("已取消" if response.success else "取消失败", response.message)
            else:
                self._set_state("录制完成" if response.success else "停止失败", response.message)

        if action == "stop" and response.success and self._close_after_stop:
            self._allow_close = True
            QTimer.singleShot(0, self.close)
            return
        if action == "stop" and not response.success:
            self._close_after_stop = False
        self._refresh_availability()

    def _start_elapsed(self) -> None:
        if self._recording_started_at is None:
            self._recording_started_at = time.monotonic()

    def _stop_elapsed(self) -> None:
        if self._recording_started_at is not None:
            self._elapsed_seconds += time.monotonic() - self._recording_started_at
            self._recording_started_at = None
        self._update_elapsed()

    def _update_elapsed(self) -> None:
        seconds = self._elapsed_seconds
        if self._recording_started_at is not None:
            seconds += time.monotonic() - self._recording_started_at
        total = int(seconds)
        hours, remainder = divmod(total, 3600)
        minutes, seconds = divmod(remainder, 60)
        self._elapsed.setText(f"录制时长：{hours:02d}:{minutes:02d}:{seconds:02d}")

    def closeEvent(self, event) -> None:  # noqa: N802
        starting = self._action == "start"
        if self._allow_close or not (self._bag_running or starting):
            event.accept()
            return
        if self._future is not None:
            QMessageBox.information(self, "请稍候", "当前操作完成后才能关闭窗口。")
            event.ignore()
            return
        answer = QMessageBox.question(
            self,
            "停止录制",
            "正在录制，是否安全停止并关闭窗口？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self._close_after_stop = True
            self._stop_recording()
        event.ignore()


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication([sys.argv[0]])
    node = DatasetGuiNode()
    window = DatasetControlWindow(node)
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
