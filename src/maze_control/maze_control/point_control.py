import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))


class PointControl(Node):

    def __init__(self):
        super().__init__('point_control')

        # =====================================
        # 1. 航点列表
        # =====================================

        self.waypoints = [
            (1.0, 2.0),
            (1.0, 2.5),
            (2.0, 2.5),
        ]

        # 当前航点索引
        self.current_waypoint_index = 0

        # =====================================
        # 2. P 控制参数
        # =====================================

        self.kp_distance = 0.5
        self.kp_angle = 1.5

        # 速度限制
        self.max_linear_speed = 0.15
        self.max_angular_speed = 0.8

        # 误差阈值
        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.25

        # =====================================
        # 3. 当前机器人状态
        # =====================================

        self.x = None
        self.y = None
        self.yaw = None

        self.finished = False

        # =====================================
        # 4. ROS 2 通信
        # =====================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 10Hz 控制循环
        self.timer = self.create_timer(
            0.1,
            self.control_callback
        )

        self.get_logger().info(
            f'航点跟踪启动，共 {len(self.waypoints)} 个目标点'
        )

        self.log_current_waypoint()

    def log_current_waypoint(self):

        if self.current_waypoint_index < len(self.waypoints):

            gx, gy = self.waypoints[
                self.current_waypoint_index
            ]

            self.get_logger().info(
                f'当前目标：航点 '
                f'{self.current_waypoint_index + 1}/'
                f'{len(self.waypoints)} '
                f'({gx:.2f}, {gy:.2f})'
            )

    # =====================================
    # 5. 获取里程计
    # =====================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # 四元数转换为偏航角 yaw
        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(sin_yaw, cos_yaw)

    # =====================================
    # 6. P 控制与航点切换
    # =====================================

    def control_callback(self):

        # 等待里程计数据
        if self.x is None or self.y is None or self.yaw is None:
            return

        # 所有航点完成
        if self.current_waypoint_index >= len(self.waypoints):

            self.publish_velocity(0.0, 0.0)

            if not self.finished:
                self.get_logger().info('全部航点跟踪完成！')
                self.finished = True

            self.timer.cancel()
            return

        # 获取当前目标点
        goal_x, goal_y = self.waypoints[
            self.current_waypoint_index
        ]

        # 计算位置误差
        dx = goal_x - self.x
        dy = goal_y - self.y

        distance_error = math.sqrt(
            dx ** 2 + dy ** 2
        )

        # =================================
        # 判断当前航点是否到达
        # =================================

        if distance_error < self.distance_tolerance:

            # 到达当前航点，先停止
            self.publish_velocity(0.0, 0.0)

            self.get_logger().info(
                f'航点 {self.current_waypoint_index + 1} 到达！'
                f'当前位置：({self.x:.3f}, {self.y:.3f})'
            )

            # 切换到下一个航点
            self.current_waypoint_index += 1

            # 输出下一个目标
            self.log_current_waypoint()

            return

        # =================================
        # 计算目标方向
        # =================================

        target_yaw = math.atan2(dy, dx)

        angle_error = normalize_angle(
            target_yaw - self.yaw
        )

        # =================================
        # P 控制
        # =================================

        # 角度误差较大时，先原地旋转
        if abs(angle_error) > self.angle_tolerance:

            angular_speed = (
                self.kp_angle * angle_error
            )

            angular_speed = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, angular_speed)
            )

            linear_speed = 0.0

        else:

            # 距离 P 控制
            linear_speed = (
                self.kp_distance * distance_error
            )

            linear_speed = min(
                linear_speed,
                self.max_linear_speed
            )

            # 角度 P 控制
            angular_speed = (
                self.kp_angle * angle_error
            )

            angular_speed = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, angular_speed)
            )

        self.publish_velocity(
            linear_speed,
            angular_speed
        )

    # =====================================
    # 7. 发布速度
    # =====================================

    def publish_velocity(self, linear_x, angular_z):

        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = PointControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # 退出前尝试发送停止指令
        if rclpy.ok():
            try:
                node.publish_velocity(0.0, 0.0)
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
