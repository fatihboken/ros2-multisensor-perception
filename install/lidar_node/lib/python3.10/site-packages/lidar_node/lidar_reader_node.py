import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar, RPLidarException
import math


class LidarReaderNode(Node):
    def __init__(self):
        super().__init__('lidar_reader_node')
        try:
            self.lidar = RPLidar('/dev/ttyUSB0', timeout=3)
            self.lidar.start_motor()
            self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.get_logger().info('Lidar Reader Node başlatıldı.')
        except RPLidarException as e:
            self.get_logger().error(f'Lidar başlatılamadı: {e}')
            raise

    def timer_callback(self):
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=500):
                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'laser'
                msg.angle_min = 0.0
                msg.angle_max = 2.0 * math.pi
                msg.angle_increment = math.radians(1.0)
                msg.time_increment = 0.0
                msg.scan_time = 0.1
                msg.range_min = 0.15
                msg.range_max = 6.0

                ranges = [0.0] * 360
                for (_, angle, distance) in scan:
                    index = int(angle)
                    if 0 <= index < 360:
                        ranges[index] = distance / 1000.0  # mm to m

                msg.ranges = ranges
                self.publisher_.publish(msg)
                break
        except RPLidarException as e:
            self.get_logger().error(f'Lidar okuma hatası: {e}')
        except Exception as e:
            self.get_logger().error(f'Genel hata: {e}')

    def destroy_node(self):
        try:
            self.lidar.stop_motor()
            self.lidar.stop()
            self.lidar.disconnect()
        except Exception as e:
            self.get_logger().warn(f'Lidar durdurulurken hata: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

