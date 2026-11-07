
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class BenchmarkTimer(Node):

    def __init__(self):
        super().__init__('benchmark_timer')

        # =====================================
        # 1. 设置起点和终点坐标
        # =====================================

        # 起点坐标
        self.start_x = 0.15
        self.start_y = 2.0

        # 终点坐标
        self.end_x = 3.7
        self.end_y = 7.0

        # 起点和终点的坐标容差
        self.start_tolerance = 0.3
        self.end_tolerance = 0.3

        # =====================================
        # 2. 计时状态
        # =====================================

        self.started = False
        self.finished = False

        self.start_time = None
        self.last_log_time = None

        # =====================================
        # 3. 订阅里程计
        # =====================================

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            '\n========== 测速节点启动 ==========\n'
            f'起点坐标：({self.start_x:.2f}, {self.start_y:.2f})\n'
            f'终点坐标：({self.end_x:.2f}, {self.end_y:.2f})\n'
            f'起点容差：{self.start_tolerance:.2f}m\n'
            f'终点容差：{self.end_tolerance:.2f}m\n'
            '等待机器人到达起点...'
        )

    # =====================================
    # 4. 判断是否进入指定坐标区域
    # =====================================

    def is_in_region(self, x, y, target_x, target_y, tolerance):

        dx = abs(x - target_x)
        dy = abs(y - target_y)

        return dx <= tolerance and dy <= tolerance

    # =====================================
    # 5. 里程计回调
    # =====================================

    def odom_callback(self, msg):

        if self.finished:
            return

        # 获取机器人当前位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 获取仿真时间
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9

        # 定期输出当前位置
        if self.last_log_time is None:
            self.last_log_time = now

        if now - self.last_log_time >= 1.0:

            self.get_logger().info(
                f'当前位置：x={x:.3f}m, y={y:.3f}m | '
                f'计时状态：{"进行中" if self.started else "等待起点"}'
            )

            self.last_log_time = now

        # =================================
        # 6. 检测起点
        # =================================

        if not self.started:

            if self.is_in_region(
                x,
                y,
                self.start_x,
                self.start_y,
                self.start_tolerance
            ):

                self.started = True
                self.start_time = now

                self.get_logger().info(
                    '\n========== 到达起点 ==========\n'
                    f'当前位置：x={x:.3f}m, y={y:.3f}m\n'
                    '开始计时！\n'
                    '=============================='
                )

        # =================================
        # 7. 检测终点
        # =================================

        else:

            if self.is_in_region(
                x,
                y,
                self.end_x,
                self.end_y,
                self.end_tolerance
            ):

                elapsed = now - self.start_time

                # 起点到终点的直线距离
                distance = (
                    (self.end_x - self.start_x) ** 2
                    + (self.end_y - self.start_y) ** 2
                ) ** 0.5

                avg_speed = (
                    distance / elapsed if elapsed > 0 else 0.0
                )

                self.finished = True

                self.get_logger().info(
                    '\n========== 到达终点！ ==========\n'
                    f'实际位置：x={x:.3f}m, y={y:.3f}m\n'
                    f'起点坐标：({self.start_x:.2f}, {self.start_y:.2f})\n'
                    f'终点坐标：({self.end_x:.2f}, {self.end_y:.2f})\n'
                    f'起终点直线距离：{distance:.3f}m\n'
                    f'运行时间：{elapsed:.3f}s\n'
                    f'平均直线速度：{avg_speed:.3f}m/s\n'
                    '================================'
                )


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
