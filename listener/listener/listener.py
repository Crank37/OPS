import rclpy
from rclpy.node import Node 
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
import math
from std_msgs.msg import Bool 
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from tf2_ros import TransformBroadcaster
import transforms3d._gohlketransforms as tf3d

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np


class listener(Node):
    def __init__(self):
        super().__init__("listener")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.Tag_Poses = {} #echte Tag Positionen

        self.tag_detection = self.create_subscription(AprilTagDetectionArray, "/detections",self.apriltag_callback, 1)
        self.publisher = self.create_publisher(Bool, "/all_tags_detected",  1)
        self.timer = self.create_timer(0.1, self.broadcast_static_transform)    #Alle TF's aus dict publishen

        self.max_distance_tag = 0.5
        #Zum kurzzeitigen Speichern des Pose für map->Offset
        self.pose_buffer_off = Pose()

        self.counter =0.

    def apriltag_callback(self, data):   
            ### Transformation map-camera_lens_link
            ### Wenn man dann noch die Transformation camer_lens_link-tag mit der oberen kombiniert, hat man die Transformation map-tag

            

            try:
                #zu Beginn wird das Dictionary geleert
                self.tag_id = {}
                self.tag_id_min_distance = {}
                #die Liste der gefundenen Tags durchiterieren, um das nähste Tag zu finden
                for detection in data.detections:
                    only_id = str(detection.id)
                    from_frame = "base_footprint" 
                    to_frame = String()
                    to_frame = "tagStandard41h12:" + only_id   #<---        NOCH BEZEICHNUNG EINFÜGEN DER TAGS
                    # Roboter -> Tag: Distanz mit Pythagoras
                    trans = self.tf_buffer.lookup_transform( from_frame, to_frame, rclpy.time.Time())
                    distance = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
                    #überprüft, ob das Tag innerhalb der Distanz liegt, damit ein guter Schätzert der Position
                    if detection.decision_margin > 20 and distance < 2.2: 
                        trans_map_tag = self.tf_buffer.lookup_transform( "map", to_frame, rclpy.time.Time())
                        self.add_trans(trans_map_tag, only_id)
            except:
                return

    

    def add_trans(self, trans, id): # die id noch mitnehmen
        new_name = String()
        new_name = "TagFrame:" + id        #der neue Name (im Dictionary gespeichert und als TF gebrooadcastet)

        self.Tag_Poses.update({new_name : trans})       





    def broadcast_static_transform(self):
            self.map_tagpose = TransformStamped()

            self.map_tagpose.header.stamp = self.get_clock().now().to_msg()

            self.map_tagpose.header.frame_id = 'map'   #rechts unten
            for key in self.Tag_Poses.keys():
                self.map_tagpose.child_frame_id = key    #Name Frame aus dictionary


                self.map_tagpose.transform.translation.x = self.Tag_Poses[key].transform.translation.x #hier einzelne Werte aus dictionary rausgefischt
                self.map_tagpose.transform.translation.y = self.Tag_Poses[key].transform.translation.y
                self.map_tagpose.transform.translation.z = self.Tag_Poses[key].transform.translation.z
                self.map_tagpose.transform.rotation.x = self.Tag_Poses[key].transform.rotation.x
                self.map_tagpose.transform.rotation.y = self.Tag_Poses[key].transform.rotation.y
                self.map_tagpose.transform.rotation.z = self.Tag_Poses[key].transform.rotation.z
                self.map_tagpose.transform.rotation.w = self.Tag_Poses[key].transform.rotation.w
                self.tf_broadcaster.sendTransform(self.map_tagpose) 

            if len(self.Tag_Poses) == 3: 
                bool_msg = Bool()
                bool_msg.data = True
                
                self.publisher.publish(bool_msg)



   




def main():
    rclpy.init()
    node = listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
