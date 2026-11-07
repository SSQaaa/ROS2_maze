import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class OpenLoopControl(Node):

    def __init__(self):
        super().__init__('open_loop_control')

        # 创建速度发布器
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 开环运动阶段：
        # (持续时间, 线速度, 角速度)
        self.motion_sequence = [
            (2.0, 0.15, 0.0),   # 向前直行 2 秒
            (1.0, 0.0, 0.0),    # 停止 1 秒
            (2.0, 0.0, 0.5),    # 原地旋转 2 秒
            (1.0, 0.0, 0.0),    # 最终停止
        ]

        self.stage = 0
        self.stage_start_time = time.monotonic()

        # 10Hz 发布速度指令
        self.timer = self.create_timer(
            0.1,
            self.control_callback
        )

        self.get_logger().info('开环运动控制节点启动')

    def control_callback(self):

        # 所有动作完成
        if self.stage >= len(self.motion_sequence):
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('所有运动动作执行完成')
            self.timer.cancel()
            return

        duration, linear_x, angular_z = self.motion_sequence[self.stage]

        # 发布当前阶段的速度
        self.publish_velocity(linear_x, angular_z)

        # 判断当前阶段是否结束
        elapsed = time.monotonic() - self.stage_start_time

        if elapsed >= duration:

            self.get_logger().info(
                f'阶段 {self.stage + 1} 完成'
            )

            # 切换到下一阶段
            self.stage += 1
            self.stage_start_time = time.monotonic()

    def publish_velocity(self, linear_x, angular_z):

        msg = Twist()

        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = OpenLoopControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 退出前发送停止指令
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
