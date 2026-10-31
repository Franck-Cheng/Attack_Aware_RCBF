#!/usr/bin/env python3
"""
scan_range_limiter_node — Protocol V2 实验基础设施.

把 defender 的 LaserScan 感知范围限制到 1.5 m:
  /scan  →  scan_range_limiter  →  /scan_defense_1p5

对每个 beam:
  if valid range <= max_range (1.5): 保留原值
  else:                               输出 +inf

输出:
  range_max = 1.5
  header.stamp / header.frame_id 完全不变
  angle_min/angle_max/angle_increment/time_increment/scan_time/range_min 不变
  intensities 对齐 (与 ranges 同长度, 逐 beam 对齐)

QoS: SENSOR_DATA (BEST_EFFORT, KEEP_LAST, depth=5), 与 Gazebo /scan 发布一致.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan


class ScanRangeLimiter(Node):
    def __init__(self):
        super().__init__("scan_range_limiter")
        self.declare_parameter("max_range", 1.5)
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_defense_1p5")
        self.max_range = self.get_parameter("max_range").value
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub = self.create_subscription(
            LaserScan, in_topic, self._cb, sensor_qos)
        self._pub = self.create_publisher(
            LaserScan, out_topic, sensor_qos)
        self.get_logger().info(
            f"scan_range_limiter: {in_topic} -> {out_topic}, max_range={self.max_range} m")

    def _cb(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header            # stamp/frame_id 完全不变
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = self.max_range      # 1.5
        n = len(msg.ranges)
        if n == 0:
            out.ranges = []
            out.intensities = []
            self._pub.publish(out)
            return
        ranges = list(msg.ranges)
        for i in range(n):
            r = ranges[i]
            if math.isnan(r) or math.isinf(r):
                continue                    # 无效/超范围保持 NaN/+inf
            if r > self.max_range:
                ranges[i] = float("inf")
        out.ranges = ranges
        # intensities 与 ranges 同长度逐 beam 对齐
        if len(msg.intensities) == n:
            out.intensities = list(msg.intensities)
        else:
            out.intensities = list(msg.intensities)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRangeLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
