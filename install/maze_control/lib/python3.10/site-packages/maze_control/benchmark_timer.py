#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class BenchmarkTimer(Node):

    def __init__(self):
        super().__init__('benchmark_timer')

        # 计时起点和终点
        self.start_x = 1.0
        self.end_x = 7.0

        self.started = False
        self.finished = False
        self.start_time = None
        self.last_x = None

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'测速节点启动：起点 x={self.start_x}m, '
            f'终点 x={self.end_x}m'
        )

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x

        # 使用仿真时间戳，而不是电脑系统时间
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9

        if self.finished:
            return

        if self.last_x is None:
            self.last_x = x
            return

        # 检测通过起点线
        if not self.started:
            if self.last_x < self.start_x <= x:
                self.started = True
                self.start_time = now

                self.get_logger().info(
                    f'通过起点！x={x:.3f}m，开始计时'
                )

        # 检测通过终点线
        elif self.last_x < self.end_x <= x:

            elapsed = now - self.start_time

            distance = self.end_x - self.start_x
            avg_speed = distance / elapsed if elapsed > 0 else 0.0

            self.finished = True

            self.get_logger().info(
                '\n========== 测速结果 ==========\n'
                f'起点：{self.start_x:.2f} m\n'
                f'终点：{self.end_x:.2f} m\n'
                f'计时距离：{distance:.2f} m\n'
                f'运行时间：{elapsed:.3f} s\n'
                f'平均进度速度：{avg_speed:.3f} m/s\n'
                '=============================='
            )

        self.last_x = x


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkTimer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
