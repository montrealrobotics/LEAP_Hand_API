#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from leap_hand.srv import LeapPosition
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading
import time
import leap_hand_utils.leap_hand_utils as lhu


class LeapErrorPlotter(Node):
    def __init__(self):
        super().__init__("leap_error_plotter")

        self.cli = self.create_client(LeapPosition, "/leap_position")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /leap_position service...")
        self.req = LeapPosition.Request()

        self.sub_cmd = self.create_subscription(
            JointState, "cmd_leap", self._receive_cmd, 10
        )

        self.cmd_pos = np.zeros(16)
        self.state_pos = np.zeros(16)
        self._lock = threading.Lock()

        self.maxlen = 200
        self.error_hist = np.zeros((self.maxlen, 16))
        self.error_fingers = np.zeros((self.maxlen, 4))
        self.t_hist = np.arange(self.maxlen)
        self.finger_groups = [
            [1, 2, 3],
            [5, 6, 7],
            [9, 10, 11],
            [13, 14, 15],
        ]
        self.finger_colors = ["r", "g", "b", "m"]
        plt.ion()
        self.fig1, (self.ax_joints, self.ax_fingers) = plt.subplots(
            2, 1, sharex=True, figsize=(10, 8)
        )

        self.lines_joints = [
            self.ax_joints.plot(self.t_hist, self.error_hist[:, i], lw=1, label=i)[0]
            for i in range(16)
        ]
        self.ax_joints.set_ylim(-0.2, 0.2)
        self.ax_joints.set_ylabel("Joint Error (degs)")
        self.ax_joints.set_title("Leap Hand Joint Errors")
        self.ax_joints.legend()

        self.lines_fingers = [
            self.ax_fingers.plot(
                self.t_hist,
                self.error_fingers[:, i],
                color=self.finger_colors[i],
                lw=2,
                label=f"Finger {i+1}",
            )[0]
            for i in range(3)
        ]
        self.lines_fingers.append(
            self.ax_fingers.plot(
                self.t_hist,
                self.error_fingers[:, 3],
                color=self.finger_colors[3],
                lw=2,
                label=f"Thumb",
            )[0]
        )
        self.ax_fingers.set_ylim(-0.5, 0.5)
        self.ax_fingers.set_xlabel("Time Steps")
        self.ax_fingers.set_ylabel("Summed Error (degs)")
        self.ax_fingers.legend()

        self.fig2, self.ax_finger_subs = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
        self.lines_finger_subs = []
        for i, ax in enumerate(self.ax_finger_subs):
            line = ax.plot(
                self.t_hist, self.error_fingers[:, i], color=self.finger_colors[i], lw=2
            )[0]
            ax.set_ylabel(f"F{i+1} err (degs)")
            ax.set_ylim(-0.5, 0.5)
            ax.grid(True)
            self.lines_finger_subs.append(line)
            if i == 4:
                line = ax.plot(
                    self.t_hist,
                    self.error_fingers[:, i],
                    color=self.finger_colors[i],
                    lw=2,
                )[0]
                ax.set_ylabel(f"Thumb err (degs)")
                ax.set_ylim(-0.5, 0.5)
                ax.grid(True)
                self.lines_finger_subs.append(line)

        self.ax_finger_subs[-1].set_xlabel("Time Steps")
        self.fig2.suptitle("Per-Finger Summed Errors")

        self.poll_thread = threading.Thread(target=self._poll_state, daemon=True)
        self.poll_thread.start()

    def _receive_cmd(self, msg: JointState):
        if len(msg.position) >= 16:
            jp = msg.position[:16]
            if jp is not None and len(jp) == 16:
                with self._lock:
                    self.cmd_pos = np.array(jp)

    def _poll_state(self):
        while rclpy.ok():
            future = self.cli.call_async(self.req)
            future.add_done_callback(self._handle_state_response)
            time.sleep(0.1)

    def _handle_state_response(self, future):
        try:
            result = future.result()
            if result and hasattr(result, "position") and len(result.position) >= 16:
                with self._lock:
                    self.state_pos = np.array(result.position[:16])
        except Exception as e:
            self.get_logger().warn(f"State response error: {e}")

    def get_current_error(self):
        with self._lock:
            error = ((self.cmd_pos - self.state_pos) / math.pi) * 180
            return error


def main(args=None):
    rclpy.init(args=args)
    node = LeapErrorPlotter()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            error = node.get_current_error()
            node.error_hist = np.roll(node.error_hist, -1, axis=0)
            node.error_hist[-1, :] = error
            node.error_fingers = np.roll(node.error_fingers, -1, axis=0)
            for i, idxs in enumerate(node.finger_groups):
                node.error_fingers[-1, i] = np.sum(np.abs(node.error_hist[-1, idxs]))

            for i in range(16):
                node.lines_joints[i].set_ydata(node.error_hist[:, i])

            for i in range(4):
                node.lines_fingers[i].set_ydata(node.error_fingers[:, i])
                node.lines_finger_subs[i].set_ydata(node.error_fingers[:, i])

            err_min = np.nanmin(node.error_hist)
            err_max = np.nanmax(node.error_hist)
            if not np.isnan(err_min) and not np.isnan(err_max):
                margin = 0.05 * (err_max - err_min if err_max != err_min else 1.0)
                node.ax_joints.set_ylim(err_min - margin, err_max + margin)

            finger_min = np.nanmin(node.error_fingers)
            finger_max = np.nanmax(node.error_fingers)
            if not np.isnan(finger_min) and not np.isnan(finger_max):
                margin = 0.05 * (
                    finger_max - finger_min if finger_max != finger_min else 1.0
                )
                node.ax_fingers.set_ylim(finger_min - margin, finger_max + margin)

            for i, ax in enumerate(node.ax_finger_subs):
                fmin = np.nanmin(node.error_fingers[:, i])
                fmax = np.nanmax(node.error_fingers[:, i])
                if not np.isnan(fmin) and not np.isnan(fmax):
                    margin = 0.05 * (fmax - fmin if fmax != fmin else 1.0)
                    ax.set_ylim(fmin - margin, fmax + margin)
            node.fig1.canvas.draw_idle()
            node.fig2.canvas.draw_idle()
            plt.pause(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
