
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class OptimizedLeftWallFollower(Node):
    """具有速度自适应和连续角速度控制的左墙跟随算法"""

    def __init__(self):
        super().__init__('wall_follow_optimized')

        # ROS2 通信接口
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # ==============================
        # 1. 控制参数
        # ==============================

        self.target_left = 0.8       # 期望左侧墙距（m）
        self.front_danger = 0.5     # 前方危险距离（m）
        self.front_slow = 1.5        # 开始减速的距离（m）

        # 线速度参数
        self.max_v = 0.5            # 最大线速度（m/s）
        self.min_v = 0.08            # 最小线速度（m/s）

        # 角速度参数
        self.kp = 1.2                # 墙距误差比例系数
        self.max_w = 1.0             # 最大角速度（rad/s）

        # 数据平滑参数
        self.alpha = 0.5             # 低通滤波系数

        # 雷达数据
        self.front = float('inf')
        self.left = float('inf')
        self.filtered_left = None
        self.scan_received = False

        # 10Hz 控制循环
        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            '优化版左墙跟随节点已启动，等待雷达数据'
        )

    def get_sector_min(self, msg, start_deg, end_deg):
        """获取指定角度范围内的最小有效距离"""

        valid_ranges = []

        start_angle = math.radians(start_deg)
        end_angle = math.radians(end_deg)

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if start_angle <= angle <= end_angle:
                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):
                    valid_ranges.append(distance)

        if not valid_ranges:
            return float('inf')

        return min(valid_ranges)

    def scan_callback(self, msg):
        """提取前方和左侧距离"""

        # 前方：0°~15° 和 345°~360°
        front_1 = self.get_sector_min(msg, 0, 15)
        front_2 = self.get_sector_min(msg, 345, 360)

        self.front = min(front_1, front_2)

        # 左侧：60°~90°
        self.left = self.get_sector_min(msg, 60, 90)
                # 分别检测几个方向
        d60 = self.get_sector_min(msg, 57, 63)
        d70 = self.get_sector_min(msg, 67, 73)
        d90 = self.get_sector_min(msg, 87, 93)

        self.get_logger().info(
            f"60°: {d60:.3f} | "
            f"70°: {d70:.3f} | "
            f"90°: {d90:.3f}"
)

        # 左侧距离低通滤波，减小雷达噪声影响
        if math.isfinite(self.left):
            if self.filtered_left is None:
                self.filtered_left = self.left
            else:
                self.filtered_left = (
                    self.alpha * self.left
                    + (1.0 - self.alpha) * self.filtered_left
                )

        self.scan_received = True

        self.get_logger().info(
            f"Front: {self.front:.3f} m | "
            f"Left: {self.left:.3f} m | "
            f"Filtered Left: "
            f"{self.filtered_left if self.filtered_left is not None else -1:.3f} m"
        )

    def calculate_linear_speed(self):
        """根据前方障碍物距离自适应调整线速度"""

        # 前方距离达到危险阈值，停止前进
        if self.front <= self.front_danger:
            return 0.0

        # 前方空间充足，使用最大速度
        if self.front >= self.front_slow:
            return self.max_v

        # 在危险距离和减速距离之间线性减速
        ratio = (
            (self.front - self.front_danger)
            / (self.front_slow - self.front_danger)
        )

        return self.min_v + ratio * (self.max_v - self.min_v)

    def calculate_angular_speed(self):
        """根据左侧墙距误差连续计算角速度"""

        # 左侧无有效数据时不进行墙距修正
        if self.filtered_left is None:
            return 0.0

        error = self.filtered_left - self.target_left

        angular_speed = self.kp * error

        # 限制角速度，避免转向过于剧烈
        angular_speed = max(
            -self.max_w,
            min(self.max_w, angular_speed)
        )

        return angular_speed

    def control_loop(self):
        """执行速度自适应和连续角速度控制"""

        twist = Twist()

        # 没有收到雷达数据时停车
        if not self.scan_received:
            self.cmd_pub.publish(twist)
            return

        # 前方危险：停止前进，原地右转
        if self.front <= self.front_danger:
            twist.linear.x = 0.0
            twist.angular.z = -self.max_w

        else:
            # 自适应线速度
            twist.linear.x = self.calculate_linear_speed()

            # 连续角速度
            twist.angular.z = self.calculate_angular_speed()

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """发送停车指令"""

        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = OptimizedLeftWallFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            try:
                node.stop_robot()
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
