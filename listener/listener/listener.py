import rclpy
from rclpy.node import Node 
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
import math

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class listener(Node):
    def __init__(self):
        super().__init__("listener")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = "map" #map frame

        self.Tag_Poses = {}

        self.tag_detection = self.create_subscription(AprilTagDetectionArray, "/detections", self.apriltag_callback, 1)

        self.timer = self.create_timer(0.1, self.broadcast_static_transform)    #Alle TF's aus dict publishen
        self.timer = self.create_timer(10.0, self.print_tag_list)               #x,y Werte Liste printen

        self.max_distance_tag = 0.5

    def apriltag_callback(self, data):    
            try:
                #zu Beginn wird das Dictionary geleert
                self.tag_id = {}
                self.tag_id_min_distance = {}

                #die Liste der gefundenen Tags durchiterieren, um das nähste Tag zu finden
                for detection in data.detections:
                    only_id = detection.id
                    from_frame = "base_footprint"
                    to_frame = String()
                    to_frame = "tagStandard41h12:" + str(only_id)   #<---        NOCH BEZEICHNUNG EINFÜGEN DER TAGS

                    # Roboter -> Tag: Distanz mit Pythagoras
                    trans = self.tf_buffer.lookup_transform( from_frame, to_frame, rclpy.time.Time())
                    distance = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

                    #überprüft, ob das Tag innerhalb der Distanz liegt, damit ein guter Schätzert der Position
                    if distance < self.max_distance_tag:
                        self.add_pose(to_frame, only_id)  #Name, ID mitsenden senden     

                    #oder mithilfe decision margin
                                        
            except:
                return
            

    def add_pose(self, real_tag_name ,id):
        new_name = String()
        new_name = "TagFrame:" + str(id)

        if new_name  not in self.Tag_Poses:
            Pose = self.tf_buffer.lookup_transform( real_tag_name, self.map, rclpy.time.Time()) 

            self.Tag_Poses.update({"x_t" : Pose.transform.translation.x, "y_t" : Pose.transform.translation.y, "z_t" : Pose.transform.translation.z , "w_o" : Pose.transform.rotation.w, "x_o" : Pose.transform.rotation.x, "y_o" : Pose.transform.rotation.y, "z_o" : Pose.transform.rotation.z})
             
    
    #printet die komplette Liste der gefundenen Tags
    def print_tag_list(self):

        #fischt sich alle INformationen aus Dictionary
        for i in self.Tag_Poses.keys():        
            
            #Dict Values in Liste abgewandelt
            abs_coordinateApril_value = list(self.Tag_Poses[i].values())

            print("----------------------------------------")
            print(" Name : " + str(i))
            print("Position: ")
            print("  X: " + str(abs_coordinateApril_value[0]))
            print("  Y: " + str(abs_coordinateApril_value[1]))
            # print("  Z: " + str(trans.transform.translation.z))
            # print("Orientation: ")
            # print("  X: " + str(trans.transform.rotation.x))
            # print("  Y: " + str(trans.transform.rotation.y))
            # print("  Z: " + str(trans.transform.rotation.z))
            # print("  W: " + str(trans.transform.rotation.w))

    def broadcast_static_transform(self):
            self.map_tagpose = TransformStamped()

            self.map_tagpose.header.stamp = self.get_clock().now().to_msg()

            self.map_tagpose.header.frame_id = 'map'   #rechts unten

            for i in self.tag_id_coordinates.keys():
                self.map_tagpose.child_frame_id = i    #Name Frame aus dictionary


                abs_coordinateApril_value = list(self.tag_id_coordinates[i].values())

                self.map_tagpose.transform.translation.x = abs_coordinateApril_value[0] #hier einzelne Werte aus dictionarya rausgefischt
                self.map_tagpose.transform.translation.y = abs_coordinateApril_value[1]
                self.map_tagpose.transform.translation.z = abs_coordinateApril_value[2]
                self.map_tagpose.transform.rotation.x = abs_coordinateApril_value[4]
                self.map_tagpose.transform.rotation.y = abs_coordinateApril_value[5]
                self.map_tagpose.transform.rotation.z = abs_coordinateApril_value[6]
                self.map_tagpose.transform.rotation.w = abs_coordinateApril_value[3]

                self.tf_broadcaster.sendTransform(self.map_tagpose)   
 
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