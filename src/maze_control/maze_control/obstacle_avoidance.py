import math
import statistics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 发布速度指令
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 定时控制，10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        # 激光雷达数据
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None
        self.last_scan_time = None

        # 控制参数
        self.safe_distance = 0.55       # 前方安全距离，单位 m
        self.resume_distance = 0.70     # 转向后恢复前进的距离，单位 m
        self.linear_speed = 0.10        # 前进速度，单位 m/s
        self.angular_speed = 0.50       # 转向速度，单位 rad/s

        # 状态参数
        self.state = 'FORWARD'
        self.turn_direction = 0.0       # 1.0 左转，-1.0 右转
        self.turn_ticks = 0
        self.min_turn_ticks = 8         # 至少转向 0.8 秒
        self.max_turn_ticks = 30        # 最多转向 3 秒

        self.get_logger().info('反应式避障节点已启动')

    def get_sector_distance(self, msg, angle_start, angle_end):
        """提取指定角度区域内有效距离的中位数。"""

        valid_ranges = []

        for i, distance in enumerate(msg.ranges):

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.degrees(angle) % 360.0

            # 判断当前激光束是否处于指定区域
            if angle_start <= angle_end:
                in_sector = angle_start <= angle <= angle_end
            else:
                in_sector = angle >= angle_start or angle <= angle_end

            if not in_sector:
                continue

            # 过滤无效距离
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            ):
                valid_ranges.append(distance)

        if not valid_ranges:
            return None

        return statistics.median(valid_ranges)

    def scan_callback(self, msg):
        """接收雷达数据并提取三个方向的距离。"""

        self.front_distance = self.get_sector_distance(
            msg, 350.0, 10.0
        )

        self.left_distance = self.get_sector_distance(
            msg, 80.0, 100.0
        )

        self.right_distance = self.get_sector_distance(
            msg, 260.0, 280.0
        )

        self.last_scan_time = self.get_clock().now()

    def publish_velocity(self, linear, angular):
        """发布机器人速度指令。"""

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def control_loop(self):
        """反应式避障控制状态机。"""

        # 检查雷达数据是否正常到达
        if self.last_scan_time is None:
            self.stop_robot()
            return

        scan_age = (
            self.get_clock().now() - self.last_scan_time
        ).nanoseconds / 1e9

        if scan_age > 1.0:
            self.stop_robot()
            self.get_logger().warning('激光雷达数据超时，机器人停车')
            return

        # 前方数据无效时，采取停车策略
        if self.front_distance is None:
            self.stop_robot()
            return

        # 状态一：正常前进
        if self.state == 'FORWARD':

            if self.front_distance > self.safe_distance:
                self.publish_velocity(self.linear_speed, 0.0)

            else:
                # 前方存在障碍物，比较左右空间
                left = (
                    self.left_distance
                    if self.left_distance is not None
                    else 0.0
                )

                right = (
                    self.right_distance
                    if self.right_distance is not None
                    else 0.0
                )

                if left > right:
                    self.turn_direction = 1.0
                    direction_text = '左转'
                elif right > left:
                    self.turn_direction = -1.0
                    direction_text = '右转'
                else:
                    # 左右距离相同或均无有效数据时，默认右转
                    self.turn_direction = -1.0
                    direction_text = '右转'

                self.state = 'TURNING'
                self.turn_ticks = 0

                # 先停车，再在后续控制周期执行转向
                self.stop_robot()

                self.get_logger().warning(
                    f'前方障碍物，距离 {self.front_distance:.3f} m，'
                    f'左侧 {left:.3f} m，右侧 {right:.3f} m，'
                    f'选择{direction_text}'
                )

        # 状态二：原地转向
        elif self.state == 'TURNING':

            self.turn_ticks += 1

            self.publish_velocity(
                0.0,
                self.turn_direction * self.angular_speed
            )

            # 至少转向一定时间，并且前方重新安全后恢复前进
            if (
                self.turn_ticks >= self.min_turn_ticks
                and self.front_distance > self.resume_distance
            ):
                self.stop_robot()
                self.state = 'FORWARD'

                self.get_logger().info(
                    f'转向完成，前方距离 {self.front_distance:.3f} m，'
                    '恢复前进'
                )

            # 防止一直转向
            elif self.turn_ticks >= self.max_turn_ticks:
                self.stop_robot()
                self.state = 'BLOCKED'

                self.get_logger().warning(
                    '转向超时，仍未找到安全方向，机器人停车'
                )

        # 状态三：无法找到安全方向，保持停车
        elif self.state == 'BLOCKED':
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()