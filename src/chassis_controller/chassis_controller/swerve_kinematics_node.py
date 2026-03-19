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
        self.declare_parameter('wheel_x', 0.215)
        self.declare_parameter('wheel_y', 0.205)
        self.declare_parameter('wheel_radius', 0.12)
        self.declare_parameter('max_steer', 1.5708)   # π/2
        self.declare_parameter('max_wheel_vel', 20.0)

        self.wheel_x = self.get_parameter('wheel_x').value
        self.wheel_y = self.get_parameter('wheel_y').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steer = self.get_parameter('max_steer').value
        self.max_wheel_vel = self.get_parameter('max_wheel_vel').value

        # 四个轮组相对车体中心的位置 [fl, fr, rl, rr]
        self.wheel_positions = [
            ( self.wheel_x,  self.wheel_y),
            ( self.wheel_x, -self.wheel_y),
            (-self.wheel_x,  self.wheel_y),
            (-self.wheel_x, -self.wheel_y),
        ]

        # 上一次的转向角
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
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        steer_angles = []
        wheel_vels = []

        for i, (rx, ry) in enumerate(self.wheel_positions):
            # 每个轮子的速度分量
            vx_i = vx - omega * ry
            vy_i = vy + omega * rx

            speed = math.hypot(vx_i, vy_i)

            # 速度极小时保持上次转向角
            if speed < 1e-4:
                steer_angles.append(self.last_angles[i])
                wheel_vels.append(0.0)
                continue

            # 目标转向角
            target_angle = math.atan2(vy_i, vx_i)

            # ── 最优化转向：超过90度时翻转180度+反转速度 ──────
            delta = self._angle_diff(target_angle, self.last_angles[i])

            if abs(delta) > math.pi / 2:
                flipped = target_angle + (math.pi if delta < 0 else -math.pi)
                # 归一化到 (-π, π]
                flipped = (flipped + math.pi) % (2 * math.pi) - math.pi
                # 只有翻转后在关节限制内才采用翻转方案
                if abs(flipped) <= self.max_steer:
                    target_angle = flipped
                    speed = -speed
                else:
                    # 翻转后也超限，直接截断到限制边界
                    target_angle = max(-self.max_steer,
                                      min(self.max_steer, target_angle))
            else:
                # 归一化
                target_angle = (target_angle + math.pi) % (2 * math.pi) - math.pi
                # 超出限制时尝试翻转
                if abs(target_angle) > self.max_steer:
                    flipped = target_angle + (math.pi if target_angle < 0 else -math.pi)
                    flipped = (flipped + math.pi) % (2 * math.pi) - math.pi
                    if abs(flipped) <= self.max_steer:
                        target_angle = flipped
                        speed = -speed
                    else:
                        target_angle = max(-self.max_steer,
                                        min(self.max_steer, target_angle))

            steer_angles.append(target_angle)
            wheel_vels.append(speed / self.wheel_radius)
            self.last_angles[i] = target_angle

        # ── 限制最大速度 ──────────────────────────────────────
        max_vel = max(abs(v) for v in wheel_vels) if wheel_vels else 0.0
        if max_vel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_vel
            wheel_vels = [v * scale for v in wheel_vels]


        # ── 调试日志 ──────────────────────────────────────────
        # self.get_logger().info(
        #     f'last: {[round(a,3) for a in self.last_angles]} '
        #     f'steer: {[round(a,3) for a in steer_angles]} '
        #     f'vel: {[round(v,3) for v in wheel_vels]}'
        # )

        # ── 发布 ──────────────────────────────────────────────
        steer_msg = Float64MultiArray()

        # ── 发布 ──────────────────────────────────────────────
        steer_msg = Float64MultiArray()
        steer_msg.data = steer_angles
        self.steer_pub.publish(steer_msg)

        wheel_msg = Float64MultiArray()
        wheel_msg.data = wheel_vels
        self.wheel_pub.publish(wheel_msg)

    def _angle_diff(self, a, b):
        """计算两角度最短差值，结果在 (-π, π]"""
        return (a - b + math.pi) % (2 * math.pi) - math.pi


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