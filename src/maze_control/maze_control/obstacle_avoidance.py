import math
import statistics

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # =====================================
        # 1. ROS 2 通信
        # =====================================

        # 订阅激光雷达，使用 Sensor Data QoS
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 发布速度指令
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 10 Hz 控制循环
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # =====================================
        # 2. 激光雷达数据
        # =====================================

        self.front_distance = None
        self.left_distance = None
        self.right_distance = None

        self.last_scan_time = None

        # =====================================
        # 3. 控制参数
        # =====================================

        self.safe_distance = 0.3       # 前方安全距离，单位 m
        self.resume_distance = 0.40     # 转向后恢复前进的距离，单位 m

        self.linear_speed = 0.10        # 前进速度，单位 m/s
        self.angular_speed = 0.50       # 转向速度，单位 rad/s

        # =====================================
        # 4. 状态参数
        # =====================================

        self.state = 'FORWARD'

        self.turn_direction = 0.0       # 1.0 左转，-1.0 右转

        self.turn_ticks = 0
        self.min_turn_ticks = 8         # 至少转向 0.8 秒
        self.max_turn_ticks = 30        # 最多转向 3 秒

        self.get_logger().info('反应式避障节点已启动')

    # =====================================
    # 5. 雷达数据处理
    # =====================================

    def get_sector_ranges(self, msg, angle_start, angle_end):
        """
        提取指定角度区域内的有效距离。

        正无穷 inf：
            视为没有检测到障碍物，使用 range_max。

        负无穷 -inf、nan：
            过滤。

        超出量程的有限数据：
            过滤。
        """

        valid_ranges = []

        for i, distance in enumerate(msg.ranges):

            # 计算当前激光束角度
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.degrees(angle) % 360.0

            # 判断当前激光束是否处于指定区域
            if angle_start <= angle_end:
                in_sector = angle_start <= angle <= angle_end
            else:
                # 处理跨越 0° 的区域，例如 350° 到 10°
                in_sector = (
                    angle >= angle_start or
                    angle <= angle_end
                )

            if not in_sector:
                continue

            # =================================
            # 处理雷达距离
            # =================================

            # 正无穷：当前方向没有检测到障碍物
            if math.isinf(distance) and distance > 0:

                valid_ranges.append(msg.range_max)

            # 有限数值：检查是否在有效量程内
            elif math.isfinite(distance):

                if msg.range_min <= distance <= msg.range_max:
                    valid_ranges.append(distance)

            # nan、负无穷及其他无效值不参与计算

        return valid_ranges

    def scan_callback(self, msg):
        """接收雷达数据并提取前方、左侧和右侧距离。"""

        # 获取三个方向的有效测距数据
        front_ranges = self.get_sector_ranges(
            msg, 350.0, 10.0
        )

        left_ranges = self.get_sector_ranges(
            msg, 80.0, 100.0
        )

        right_ranges = self.get_sector_ranges(
            msg, 260.0, 280.0
        )

        # 前方：使用最小距离，检测最近障碍物
        self.front_distance = (
            min(front_ranges) if front_ranges else None
        )

        # 左侧：使用中位数，降低异常点影响
        self.left_distance = (
            statistics.median(left_ranges)
            if left_ranges else None
        )

        # 右侧：使用中位数，降低异常点影响
        self.right_distance = (
            statistics.median(right_ranges)
            if right_ranges else None
        )

        # 记录最后一次收到雷达数据的时间
        self.last_scan_time = self.get_clock().now()

    # =====================================
    # 6. 速度控制
    # =====================================

    def publish_velocity(self, linear, angular):
        """发布机器人速度指令。"""

        msg = Twist()

        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        """机器人停车。"""

        self.publish_velocity(0.0, 0.0)

    # =====================================
    # 7. 避障控制状态机
    # =====================================

    def control_loop(self):
        """反应式避障控制状态机。"""

        # 检查雷达数据是否正常到达
        if self.last_scan_time is None:
            self.stop_robot()
            return

        # 检查雷达数据是否超时
        scan_age = (
            self.get_clock().now() - self.last_scan_time
        ).nanoseconds / 1e9

        if scan_age > 1.0:
            self.stop_robot()
            self.get_logger().warning(
                '激光雷达数据超时，机器人停车'
            )
            return

        # 前方数据无效时，采取停车策略
        if self.front_distance is None:
            self.stop_robot()
            return

        # =================================
        # 状态一：正常前进
        # =================================

        if self.state == 'FORWARD':

            if self.front_distance > self.safe_distance:

                self.publish_velocity(
                    self.linear_speed,
                    0.0
                )

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

        # =================================
        # 状态二：原地转向
        # =================================

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
                    f'转向完成，前方距离 '
                    f'{self.front_distance:.3f} m，'
                    '恢复前进'
                )

            # 防止一直转向
            elif self.turn_ticks >= self.max_turn_ticks:

                self.stop_robot()
                self.state = 'BLOCKED'

                self.get_logger().warning(
                    '转向超时，仍未找到安全方向，机器人停车'
                )

        # =================================
        # 状态三：无法找到安全方向
        # =================================

        elif self.state == 'BLOCKED':

            self.stop_robot()

    # =====================================
    # 8. 主程序
    # =====================================


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # 退出前发送停止指令
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
