import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import os
import json

class MapTracker:
    def __init__(self, save_dir='maps'):
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)
        self.step = 0

    def save_map(self, points):
        filename = os.path.join(self.save_dir, f'map_step_{self.step}.json')
        with open(filename, 'w') as f:
            json.dump(points, f)
        self.step += 1
        print(f'Harita {filename} olarak kaydedildi.')

class MapBuilderNode(Node):
    def __init__(self):
        super().__init__('map_builder_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.get_logger().info('Map Builder Node başlatıldı.')

        # Matplotlib başlat
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ln, = self.ax.plot([], [], 'b.', markersize=2)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.fig.canvas.draw()  # Renderer önbelleği oluştur
        plt.show(block=False)

        self.points = []
        self.tracker = MapTracker()

    def listener_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        angles = angle_min + np.arange(len(ranges)) * angle_increment

        # Geçerli verileri filtrele
        valid = np.logical_and(ranges > msg.range_min, ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        self.points.extend(zip(x.tolist(), y.tolist()))  # Harita için biriktir

        self.ln.set_data(x, y)
        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.ln)
        self.fig.canvas.flush_events()
        plt.pause(0.001)

        # Haritayı dosyaya kaydet
        self.tracker.save_map(self.points)


def main(args=None):
    rclpy.init(args=args)
    node = MapBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

