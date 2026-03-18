#!/usr/bin/env python3
"""
Swerve Drive 运动学节点
订阅 /cmd_vel，解算后发布到：
  - /steering_position_controller/commands  (四轮转角)
  - /wheel_velocity_controller/commands     (四轮速度)

关节顺序：[fl, fr, rl, rr]
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class SwerveDriveKinematics(Node):

    def __init__(self):
        super().__init__('swerve_kinematics')

        # ── 底盘参数 ──────────────────────────────────────────
        self.declare_parameter('wheel_x', 0.215)      # 轮组到车体中心的前后距离 (m)
        self.declare_parameter('wheel_y', 0.205)      # 轮组到车体中心的左右距离 (m)
        self.declare_parameter('wheel_radius', 0.12)  # 车轮半径 (m)
        self.declare_parameter('max_steer', 1.5708)   # 转向关节最大角度 (rad) = π/2
        self.declare_parameter('max_wheel_vel', 20.0) # 车轮最大角速度 (rad/s)

        self.wheel_x = self.get_parameter('wheel_x').value
        self.wheel_y = self.get_parameter('wheel_y').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steer = self.get_parameter('max_steer').value
        self.max_wheel_vel = self.get_parameter('max_wheel_vel').value

        # 四个轮组相对车体中心的位置 [fl, fr, rl, rr]
        self.wheel_positions = [
            ( self.wheel_x,  self.wheel_y),   # fl
            ( self.wheel_x, -self.wheel_y),   # fr
            (-self.wheel_x,  self.wheel_y),   # rl
            (-self.wheel_x, -self.wheel_y),   # rr
        ]

        # 上一次的转向角（用于最小化转向变化量）
        self.last_angles = [0.0, 0.0, 0.0, 0.0]

        # ── 发布者 ────────────────────────────────────────────
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_position_controller/commands',
            10
        )
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )

        # ── 订阅者 ────────────────────────────────────────────
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Swerve kinematics node started.')
        self.get_logger().info(
            f'wheel_x={self.wheel_x}, wheel_y={self.wheel_y}, '
            f'wheel_radius={self.wheel_radius}'
        )

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x    # 前进速度 (m/s)
        vy = msg.linear.y    # 横移速度 (m/s)
        omega = msg.angular.z  # 旋转角速度 (rad/s)

        steer_angles = []
        wheel_vels = []

        for i, (rx, ry) in enumerate(self.wheel_positions):
            # 每个轮子的速度分量
            vx_i = vx - omega * ry
            vy_i = vy + omega * rx

            # 轮子线速度大小
            speed = math.hypot(vx_i, vy_i)

            # 如果速度极小，保持上次转向角不变，速度归零
            if speed < 1e-4:
                steer_angles.append(self.last_angles[i])
                wheel_vels.append(0.0)
                continue

            # 目标转向角
            target_angle = math.atan2(vy_i, vx_i)

            # ── 最优化转向方向 ──────────────────────────────
            # 计算与上次角度的差值
            delta = self._angle_diff(target_angle, self.last_angles[i])

            if abs(delta) > math.pi / 2:
                # 转动超过90度时，反转车轮速度，少转一半
                if delta > 0:
                    target_angle -= math.pi
                else:
                    target_angle += math.pi
                speed = -speed  # 反转速度

            # ── 限制在关节范围内 ─────────────────────────────
            target_angle = self._clamp_angle(target_angle)

            steer_angles.append(target_angle)
            wheel_vels.append(speed / self.wheel_radius)
            self.last_angles[i] = target_angle

        # ── 限制最大速度 ─────────────────────────────────────
        max_vel = max(abs(v) for v in wheel_vels) if wheel_vels else 0.0
        if max_vel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_vel
            wheel_vels = [v * scale for v in wheel_vels]

        # ── 发布 ─────────────────────────────────────────────
        steer_msg = Float64MultiArray()
        steer_msg.data = steer_angles
        self.steer_pub.publish(steer_msg)

        wheel_msg = Float64MultiArray()
        wheel_msg.data = wheel_vels
        self.wheel_pub.publish(wheel_msg)

    def _angle_diff(self, a, b):
        """计算两个角度的最短差值，结果在 (-π, π]"""
        diff = (a - b + math.pi) % (2 * math.pi) - math.pi
        return diff

    def _clamp_angle(self, angle):
        """将角度归一化到 (-π, π]，再限制在转向范围内"""
        # 归一化
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        # 限制范围
        return max(-self.max_steer, min(self.max_steer, angle))


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()