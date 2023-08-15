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

        self.map = "map" #map frame

        self.Tag_Poses = {} #echte Tag Positionen
        self.Tag_PosesOff = {}  #Tag posen mit z-Offset

        self.tag_detection = self.create_subscription(AprilTagDetectionArray, "/detections",self.apriltag_callback, 1)
        self.publisher = self.create_publisher(Bool, "/all_tags_detected",  1)
        self.timer = self.create_timer(0.1, self.broadcast_static_transform)    #Alle TF's aus dict publishen
        # self.timer = self.create_timer(10.0, self.print_tag_list)               #x,y Werte Liste printen

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
                    to_frame = "tag16h5:" + only_id   #<---        NOCH BEZEICHNUNG EINFÜGEN DER TAGS

                    # Roboter -> Tag: Distanz mit Pythagoras
                    trans = self.tf_buffer.lookup_transform( from_frame, to_frame, rclpy.time.Time())
                    distance = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
                    #überprüft, ob das Tag innerhalb der Distanz liegt, damit ein guter Schätzert der Position
                    if detection.decision_margin > 20 and distance < 2.2: #distance < self.max_distance_tag:
                        # self.add_pose(to_frame, only_id)  #Name, ID mitsenden senden     
                        trans_map_tag = self.tf_buffer.lookup_transform( "map", to_frame, rclpy.time.Time())
                        self.add_trans(trans_map_tag, only_id)
            except:
                return
##### wie starte ich das zufällige umherfahren?
##### tag-Größe spätestens in echt anpassen

    

    def add_trans(self, trans, id): # die id noch mitnehmen
        new_name = String()
        new_name = "TagFrame:" + id        #der neue Name (im Dictionary gespeichert und als TF gebrooadcastet)
        ######### erstmal die Abfrage, ob vorhanden überspringen. So wird die Stelle stets überschrieben

        self.Tag_Poses.update({new_name : trans})       



    # #Fügt Pose in die Liste ein
    # def add_pose(self, real_tag_name ,id):
    #     new_name = String()
    #     new_name = "TagFrame:" + str(id)        #der neue Name (im Dictionary gespeichert und als TF gebrooadcastet)

    #     if new_name  not in self.Tag_Poses:
    #         #In die Liste (echte Pose)
    #         Pose = self.tf_buffer.lookup_transform( real_tag_name, self.map, rclpy.time.Time()) 
    #         self.Tag_Poses.update({new_name : {"x_t" : Pose.transform.translation.x, "y_t" : Pose.transform.translation.y, "z_t" : Pose.transform.translation.z , "w_o" : Pose.transform.rotation.w, "x_o" : Pose.transform.rotation.x, "y_o" : Pose.transform.rotation.y, "z_o" : Pose.transform.rotation.z}})


    #         #in die Liste zum navigieren (inklusive Offset, damit keine Kollision)
    #         PoseOff = self.calculate_offset(Pose)
    #         self.Tag_PosesOff.update({new_name : {"x_t" : PoseOff.transform.translation.x, "y_t" : PoseOff.transform.translation.y, "z_t" : PoseOff.transform.translation.z , "w_o" : PoseOff.transform.rotation.w, "x_o" : PoseOff.transform.rotation.x, "y_o" : PoseOff.transform.rotation.y, "z_o" : PoseOff.transform.rotation.z}})





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
                print("map-tag translation", self.map_tagpose.transform.translation)
                self.tf_broadcaster.sendTransform(self.map_tagpose) 

            if len(self.Tag_Poses) == 3: #5 dann eigentlich 5
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
