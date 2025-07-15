import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class MultiCameraHumanHighlighter(Node):
    def __init__(self):
        super().__init__('multi_camera_human_highlighter')
        self.bridge = CvBridge()
        self.active_index = 0  # İlk başta Kamera 0 seçili

        # 3 kamera başlatılıyor
        self.captures = [
            cv2.VideoCapture(0),
            cv2.VideoCapture(2),
            cv2.VideoCapture(4)
        ]

        # Kamera ayarları
        for i, cap in enumerate(self.captures):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            cap.set(cv2.CAP_PROP_FPS, 5)
            if not cap.isOpened():
                self.get_logger().warn(f"Kamera {i} açılamadı.")

        # HOG + SVM insan dedektörü
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.publisher = self.create_publisher(Image, 'processed_image', 10)
        self.timer = self.create_timer(0.2, self.update_frames)  # 5 FPS

        for i in range(3):
            window_name = f"Kamera {i}"
            cv2.namedWindow(window_name)
            cv2.setMouseCallback(window_name, self.make_mouse_callback(i))

        self.get_logger().info("🖱️ İnsan tespiti yapılacak kamerayı seçmek için pencereye tıklayın.")

    def make_mouse_callback(self, index):
        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.active_index = index
                self.get_logger().info(f"✔️ Kamera {index} seçildi (insan algılama aktif)")
        return on_mouse

    def update_frames(self):
        for i, cap in enumerate(self.captures):
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f"Kamera {i} görüntü alamadı.")
                continue

            # Eğer bu seçili kamera ise, insan algıla
            if i == self.active_index:
                rects, _ = self.hog.detectMultiScale(frame, winStride=(4, 4), padding=(8, 8), scale=1.05)

                if len(rects) > 0:
                    for (x, y, w, h) in rects:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    self.get_logger().info(f"👤 Kamera {i}: {len(rects)} insan bulundu.")
                else:
                    self.get_logger().info(f"🙅 Kamera {i}: insan yok.")

                # Bu kamera işlenmiş olarak yayınlanır
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher.publish(msg)

            # Her kamera gösterilir (işlenen de dahil)
            cv2.imshow(f"Kamera {i}", frame)

        cv2.waitKey(1)

    def destroy_node(self):
        for cap in self.captures:
            cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraHumanHighlighter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

