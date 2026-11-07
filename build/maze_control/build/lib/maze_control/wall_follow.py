import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LeftWallFollower(Node):
    """基于激光雷达的左墙跟随控制"""

    def __init__(self):
        super().__init__('wall_follow')

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

        # 控制参数
        self.target_left = 0.4       # 期望左侧墙距（m）
        self.front_danger = 0.35     # 前方危险距离（m）

        self.v = 0.15                # 直行速度（m/s）
        self.w = 0.6                 # 原地转弯角速度（rad/s）

        # 左右微调角速度
        self.left_adjust = 0.4
        self.right_adjust = -0.3

        # 雷达数据
        self.front = float('inf')
        self.left = float('inf')
        self.scan_received = False

        # 10Hz 控制循环
        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info('左墙跟随节点已启动，等待雷达数据')

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
        """处理雷达数据，提取前方和左侧距离"""

        # 前方：0°~15° 和 345°~360°
        front_1 = self.get_sector_min(msg, 0, 15)
        front_2 = self.get_sector_min(msg, 345, 360)

        self.front = min(front_1, front_2)

        # 左侧：60°~90°
        self.left = self.get_sector_min(msg, 60, 90)

        self.scan_received = True

    def control_loop(self):
        """根据雷达距离执行反应式墙跟随控制"""

        twist = Twist()

        # 没有收到雷达数据时保持停车
        if not self.scan_received:
            self.cmd_pub.publish(twist)
            return

        # 情况一：前方障碍物太近，原地右转
        if self.front < self.front_danger:
            twist.linear.x = 0.0
            twist.angular.z = -self.w

        # 情况二：左侧墙距离太远，向左微调
        elif self.left > self.target_left + 0.1:
            twist.linear.x = self.v
            twist.angular.z = self.left_adjust

        # 情况三：左侧墙距离太近，向右微调
        elif self.left < self.target_left - 0.1:
            twist.linear.x = self.v
            twist.angular.z = self.right_adjust

        # 情况四：左侧距离合适，直线前进
        else:
            twist.linear.x = self.v
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """发送停车指令"""

        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = LeftWallFollower()

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