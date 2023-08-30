import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from collections import defaultdict
import apriltag
import threading

class AprilTagScorer(Node):
    def __init__(self):
        super().__init__('april_tag_scorer')
        self.subscription = self.create_subscription(
            String,
            'apriltag_ids',
            self.tag_id_callback,
            10
        )
        self.tag_dict = defaultdict(bool)  # Dictionary, um doppelte Tags zu verhindern
        self.found_tags = 0
        self.score = 0
        self.start_time = None
        self.detector = apriltag.Detector()
        self.is_timing = False

    def tag_id_callback(self, msg):
        if not self.is_timing:
            self.start_time = time.time()
            self.is_timing = True

        tag_ids = msg.data.split()  # Annahme: IDs sind durch Leerzeichen getrennt
        for tag_id_str in tag_ids:
            try:
                tag_id = int(tag_id_str)
                if tag_id in self.detector.tag41h12_family:
                    if tag_id not in self.tag_dict or not self.tag_dict[tag_id]:
                        self.tag_dict[tag_id] = True
                        self.found_tags += 1
                        self.score += 10
                        elapsed_time = time.time() - self.start_time
                        self.get_logger().info(f"Tag ID {tag_id} gefunden. Benötigte Zeit: {elapsed_time:.2f} Sekunden. Aktueller Score: {self.score}")
                        if self.found_tags == 6:
                            self.end_measurement()
            except ValueError:
                pass

    def end_measurement(self):
        total_time = time.time() - self.start_time
        final_score = self.score + 10  # Zusätzliche 10 Punkte für das Erreichen des Ziels
        self.get_logger().info(f"Ziel erreicht!!! Gesamtzeit: {total_time:.2f} Sekunden. Endgültiger Score: {final_score}")
        self.start_countdown()

    def start_countdown(self):
        countdown_thread = threading.Thread(target=self.countdown_thread)
        countdown_thread.start()

    def countdown_thread(self):
        self.get_logger().info("Zeit abgelaufen (Time's up)!")
        for i in range(3, 0, -1):
            time.sleep(1)
            self.get_logger().info(f"{i}...")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagScorer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
