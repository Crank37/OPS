import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from collections import defaultdict
import apriltag



class AprilTagCounter(Node):
    def __init__(self):
        super().__init__('april_tag_counter')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.tag_dict = defaultdict(bool)  # Dictionary, um doppelte Tags zu verhindern
        self.found_tags = 0
        self.start_time = None
        self.options = apriltag.DetectorOptions(families='tag16h5')
        self.detector = apriltag.Detector(self.options)

    def image_callback(self, msg):
        if self.found_tags < 6:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            tags = self.detector.detect(gray_image)
            
            for tag in tags:
                tag_id = tag.tag_id

                if tag_id is not None and not self.tag_dict[tag_id]:
                    self.tag_dict[tag_id] = True
                    self.found_tags += 1
                    if self.found_tags == 1:
                        self.start_time = time.time()

                if self.found_tags == 6:
                    self.end_measurement()

    def end_measurement(self):
        end_time = time.time()
        total_time = end_time - self.start_time

        self.get_logger().info(f"Finish!!! Gesamtzeit: {total_time:.2f} Sekunden")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagCounter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
