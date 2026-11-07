import csv
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomRecorder(Node):

    def __init__(self):
        super().__init__('odom_recorder')

        self.file = open(
            'odom_l2.csv',
            'w',
            newline='',
            encoding='utf-8'
        )

        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time',
            'x',
            'y',
            'yaw',
            'linear_x',
            'angular_z'
        ])

        self.start_time = time.monotonic()
        self.last_record_time = 0.0
        self.count = 0

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('里程计记录器已启动，正在记录 /odom')

    def odom_callback(self, msg):

        current_time = time.monotonic()
        elapsed = current_time - self.start_time

        # 每隔约 0.1 秒记录一次，避免数据过多
        if elapsed - self.last_record_time < 0.1:
            return

        self.last_record_time = elapsed

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # 四元数转换为偏航角 yaw
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cos_yaw = 1.0 - 2.0 * (
            orientation.y ** 2
            + orientation.z ** 2
        )

        yaw = math.atan2(sin_yaw, cos_yaw)

        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z

        self.writer.writerow([
            round(elapsed, 3),
            round(position.x, 4),
            round(position.y, 4),
            round(yaw, 4),
            round(linear_x, 4),
            round(angular_z, 4)
        ])

        self.count += 1

        if self.count % 50 == 0:
            self.file.flush()
            self.get_logger().info(
                f'已记录 {self.count} 组数据'
            )

    def destroy_node(self):
        self.file.flush()
        self.file.close()
        self.get_logger().info(
            f'记录结束，共保存 {self.count} 组数据'
        )
        super().destroy_node()


def main():
    rclpy.init()
    node = OdomRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()