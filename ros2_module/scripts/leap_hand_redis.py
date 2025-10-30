#!/usr/bin/env python3

import os
import json
import time
import numpy as np
import redis
import rclpy
import socket
import subprocess
import pickle
from rclpy.node import Node
from sensor_msgs.msg import JointState
from leap_hand.srv import LeapPosition

REDIS_KEY = "right_leap_action"
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6669


def start_redis_server(port=REDIS_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((REDIS_HOST, port))
        sock.close()
        print(f"[INFO] Redis already running on port {port}")
    except ConnectionRefusedError:
        print(f"[INFO] No Redis server found on port {port}. Starting one...")
        subprocess.Popen(
            ["redis-server", "--port", str(port)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(1.5)
        print(f"[INFO] Redis server started on port {port}")


class LeapHandRedis(Node):
    def __init__(self):
        super().__init__("leap_hand_redis")

        self.pub_hand = self.create_publisher(JointState, "/cmd_allegro", 10)
        self.req = LeapPosition.Request()
        self.loop_timer = self.create_timer(0.05, self.loop)
        start_redis_server(REDIS_PORT)
        self.redis = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)

        self.get_logger().info(f"Connected to Redis at {REDIS_HOST}:{REDIS_PORT}")
        self.last_cmd = None

    def publish_joint_state(self, joint_positions):
        msg = JointState()
        msg.position = joint_positions
        self.pub_hand.publish(msg)

    def parse_command(self, raw_val):
        if raw_val is None:
            return None
        try:
            cmd = pickle.loads(raw_val)
            return list(map(float, cmd))
        except Exception as e:
            self.get_logger().warn(f"Failed to unpickle Redis data: {e}")
            return None

    def loop(self):
        try:
            raw_val = self.redis.get(REDIS_KEY)
            if raw_val and raw_val != self.last_cmd:
                joint_cmd = self.parse_command(raw_val)
                if joint_cmd:
                    self.get_logger().debug(f"Received new Redis command: {joint_cmd} ...")
                    self.publish_joint_state(joint_cmd)
                    self.last_cmd = raw_val
        except redis.exceptions.ConnectionError as e:
            self.get_logger().error(f"Redis connection error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LeapHandRedis()
    rclpy.spin(node)
    try:
        node.get_logger().info("Starting Leap Hand Redis bridge...")
        node.loop()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

