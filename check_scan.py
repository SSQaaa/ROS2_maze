import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanChecker(Node):

    def __init__(self):
        super().__init__('scan_checker')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('正在等待雷达数据...')
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanChecker(Node):

    def __init__(self):
        super().__init__('scan_checker')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('正在等待雷达数据...')

    def get_min_distance(self, msg, start_deg, end_deg):
        values = []
        total = 0

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            if start_deg <= angle_deg <= end_deg:
                total += 1

                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):
                    values.append(distance)

        if not values:
            return float('inf'), total, 0

        return min(values), total, len(values)

    def scan_callback(self, msg):

        # 前方：0°~15° 和 345°~360°
        front_1, front_total_1, front_valid_1 = self.get_min_distance(msg, 0, 15)
        front_2, front_total_2, front_valid_2 = self.get_min_distance(msg, 345, 360)

        front = min(front_1, front_2)
        front_total = front_total_1 + front_total_2
        front_valid = front_valid_1 + front_valid_2

        # 左侧：60°~90°
        left, left_total, left_valid = self.get_min_distance(msg, 60, 90)

        # 右侧：270°~300°
        right, right_total, right_valid = self.get_min_distance(msg, 270, 300)

        print('\n========== 雷达数据 ==========')
        print(f'雷达角度范围：{math.degrees(msg.angle_min):.1f}° ~ '
            f'{math.degrees(msg.angle_max):.1f}°')

        print(f'前方：{front:.3f} m | 采样点 {front_total} | 有效点 {front_valid}')
        print(f'左侧：{left:.3f} m | 采样点 {left_total} | 有效点 {left_valid}')
        print(f'右侧：{right:.3f} m | 采样点 {right_total} | 有效点 {right_valid}')
        print('==============================')

def main():
    rclpy.init()
    node = ScanChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    def get_min_distance(self, msg, start_deg, end_deg):
        values = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            if start_deg <= angle_deg <= end_deg:
                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):
                    values.append(distance)

        if not values:
            return float('inf')

        return min(values)

    def scan_callback(self, msg):

        front = self.get_min_distance(msg, -15, 15)
        left = self.get_min_distance(msg, 60, 90)
        right = self.get_min_distance(msg, -90, -60)

        print('\n========== 雷达数据 ==========')
        print(f'雷达角度范围：{math.degrees(msg.angle_min):.1f}° ~ '
              f'{math.degrees(msg.angle_max):.1f}°')
        print(f'前方距离：{front:.3f} m')
        print(f'左侧距离：{left:.3f} m')
        print(f'右侧距离：{right:.3f} m')
        print('==============================')

def main():
    rclpy.init()
    node = ScanChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
