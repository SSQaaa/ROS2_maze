import math
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarProcess(Node):

    def __init__(self):
        super().__init__('lidar_process')

        # 订阅激光雷达话题
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 定时输出，避免每帧都打印
        self.timer = self.create_timer(0.5, self.print_distances)

        self.front_distance = None
        self.left_distance = None
        self.right_distance = None

        self.get_logger().info('激光雷达数据处理节点已启动')

    def get_sector_distance(self, msg, angle_start, angle_end):
        """
        提取指定角度区域内的有效距离，并计算中位数。

        angle_start、angle_end：角度，单位为度。
        支持跨越 0° 的角度区域。
        """

        valid_ranges = []

        for i, distance in enumerate(msg.ranges):

            # 根据数组索引计算当前激光束角度
            angle = msg.angle_min + i * msg.angle_increment

            # 转换为 0~360°
            angle = math.degrees(angle) % 360.0

            # 判断是否处于目标角度区域
            if angle_start <= angle_end:
                in_sector = angle_start <= angle <= angle_end
            else:
                # 处理跨越 0° 的区域，例如 350°~10°
                in_sector = angle >= angle_start or angle <= angle_end

            if not in_sector:
                continue

            # 过滤 inf、nan 和超出量程的距离
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            ):
                valid_ranges.append(distance)

        if not valid_ranges:
            return None

        # 使用中位数降低个别异常测量值的影响
        return statistics.median(valid_ranges)

    def scan_callback(self, msg):

        # 前方：350°~10°
        self.front_distance = self.get_sector_distance(
            msg, 350.0, 10.0
        )

        # 左侧：80°~100°
        self.left_distance = self.get_sector_distance(
            msg, 80.0, 100.0
        )

        # 右侧：260°~280°
        self.right_distance = self.get_sector_distance(
            msg, 260.0, 280.0
        )

    def print_distances(self):

        def format_distance(distance):
            if distance is None:
                return '无有效回波'
            return f'{distance:.3f} m'

        self.get_logger().info(
            f'前方: {format_distance(self.front_distance)} | '
            f'左侧: {format_distance(self.left_distance)} | '
            f'右侧: {format_distance(self.right_distance)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcess()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()