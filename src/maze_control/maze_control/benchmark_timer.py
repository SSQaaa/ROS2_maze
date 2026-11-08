#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class BenchmarkTimer(Node):

    def __init__(self, end_x, end_y):
        super().__init__('benchmark_timer')

        # =====================================
        # 1. 设置起点和终点坐标
        # =====================================

        # 起点坐标
        self.start_x = 0.0
        self.start_y = 1.0

        # 终点坐标：由启动前交互输入
        self.end_x = end_x
        self.end_y = end_y

        # 起点和终点容差
        self.start_tolerance = 0.3
        self.end_tolerance = 0.3

        # 坐标轴判断方式：
        # 'both'：同时判断 X、Y
        # 'x'：只判断 X
        # 'y'：只判断 Y
        self.start_axis = 'both'
        self.end_axis = 'y'

        # 终点判断模式：
        # True：单向越过终点即可结束（适合沿 Y 正方向行驶）
        # False：在目标坐标容差范围内结束
        self.end_one_way = False

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
            f'起点判断轴：{self.start_axis}\n'
            f'终点判断轴：{self.end_axis}\n'
            f'终点单向判断：{self.end_one_way}\n'
            '等待机器人到达起点...'
        )

    # =====================================
    # 4. 判断是否到达起点区域
    # =====================================

    def is_in_region(
        self, x, y, target_x, target_y, tolerance, axis='both'
    ):

        if axis == 'x':
            return abs(x - target_x) <= tolerance

        elif axis == 'y':
            return abs(y - target_y) <= tolerance

        elif axis == 'both':
            dx = abs(x - target_x)
            dy = abs(y - target_y)
            return dx <= tolerance and dy <= tolerance

        else:
            self.get_logger().error(
                f'无效的坐标轴判断方式：{axis}'
            )
            return False

    # =====================================
    # 5. 判断是否到达终点
    # =====================================

    def is_at_end(self, x, y):

        # 单向越过终点判断
        if self.end_one_way:

            if self.end_axis == 'x':
                return x >= self.end_x

            elif self.end_axis == 'y':
                return y >= self.end_y

            else:
                self.get_logger().error(
                    '单向判断只支持 x 或 y 轴'
                )
                return False

        # 普通容差判断
        return self.is_in_region(
            x,
            y,
            self.end_x,
            self.end_y,
            self.end_tolerance,
            self.end_axis
        )

    # =====================================
    # 6. 里程计回调
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
        # 7. 检测起点
        # =================================

        if not self.started:

            if self.is_in_region(
                x,
                y,
                self.start_x,
                self.start_y,
                self.start_tolerance,
                self.start_axis
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
        # 8. 检测终点
        # =================================

        else:

            if self.is_at_end(x, y):

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
                    f'终点判断轴：{self.end_axis}\n'
                    f'起终点直线距离：{distance:.3f}m\n'
                    f'运行时间：{elapsed:.3f}s\n'
                    f'平均直线速度：{avg_speed:.3f}m/s\n'
                    '================================'
                )


def main(args=None):

    # =====================================
    # 9. 启动前交互输入终点坐标
    # =====================================

    print('\n========== 迷宫测速参数设置 ==========')
    print('请输入本次测试的终点坐标：')

    while True:
        try:
            end_x = float(input('终点 X 坐标（m）：'))
            end_y = float(input('终点 Y 坐标（m）：'))
            break

        except ValueError:
            print('输入无效！请输入数字，例如：3.7')

    print(f'\n本次测试终点：({end_x:.2f}, {end_y:.2f})')
    print('终点判断方式：只判断 Y 轴')
    print('终点判断条件：Y >= 终点 Y 坐标')
    print('======================================\n')

    # 初始化 ROS2
    rclpy.init(args=args)

    node = BenchmarkTimer(end_x, end_y)

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
