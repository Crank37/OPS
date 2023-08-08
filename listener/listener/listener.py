import rclpy
from rclpy.node import Node 
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
import math

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

        self.tag_detection = self.create_subscription(AprilTagDetectionArray, "/detections", self.apriltag_callback, 1)

        self.timer = self.create_timer(0.1, self.broadcast_static_transform)    #Alle TF's aus dict publishen
        # self.timer = self.create_timer(10.0, self.print_tag_list)               #x,y Werte Liste printen

        self.max_distance_tag = 0.5

        #Zum kurzzeitigen Speichern des Pose für map->Offset
        self.pose_buffer_off = Pose()

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
        # if new_name  not in self.Tag_Poses:
            #In die Liste (echte Pose)
        self.Tag_Poses.update({new_name : trans})       
            # Pose = trans # self.tf_buffer.lookup_transform( real_tag_name, self.map, rclpy.time.Time()) 
            # self.Tag_Poses.update({new_name : {"x_t" : Pose.transform.translation.x, "y_t" : Pose.transform.translation.y, "z_t" : Pose.transform.translation.z , "w_o" : Pose.transform.rotation.w, "x_o" : Pose.transform.rotation.x, "y_o" : Pose.transform.rotation.y, "z_o" : Pose.transform.rotation.z}})

            # #in die Liste zum navigieren (inklusive Offset, damit keine Kollision)
            # PoseOff = self.calculate_offset(Pose)
            # self.Tag_PosesOff.update({new_name : {"x_t" : PoseOff.transform.translation.x, "y_t" : PoseOff.transform.translation.y, "z_t" : PoseOff.transform.translation.z , "w_o" : PoseOff.transform.rotation.w, "x_o" : PoseOff.transform.rotation.x, "y_o" : PoseOff.transform.rotation.y, "z_o" : PoseOff.transform.rotation.z}})




    #Fügt Pose in die Liste ein
    def add_pose(self, real_tag_name ,id):
        new_name = String()
        new_name = "TagFrame:" + str(id)        #der neue Name (im Dictionary gespeichert und als TF gebrooadcastet)

        if new_name  not in self.Tag_Poses:
            #In die Liste (echte Pose)
            Pose = self.tf_buffer.lookup_transform( real_tag_name, self.map, rclpy.time.Time()) 
            self.Tag_Poses.update({new_name : {"x_t" : Pose.transform.translation.x, "y_t" : Pose.transform.translation.y, "z_t" : Pose.transform.translation.z , "w_o" : Pose.transform.rotation.w, "x_o" : Pose.transform.rotation.x, "y_o" : Pose.transform.rotation.y, "z_o" : Pose.transform.rotation.z}})


            #in die Liste zum navigieren (inklusive Offset, damit keine Kollision)
            PoseOff = self.calculate_offset(Pose)
            self.Tag_PosesOff.update({new_name : {"x_t" : PoseOff.transform.translation.x, "y_t" : PoseOff.transform.translation.y, "z_t" : PoseOff.transform.translation.z , "w_o" : PoseOff.transform.rotation.w, "x_o" : PoseOff.transform.rotation.x, "y_o" : PoseOff.transform.rotation.y, "z_o" : PoseOff.transform.rotation.z}})



    #Berechnet Offset mit Matrixmulitplikation (map -> Tag -> Offset (z-Richtung vom Tag))
    def calculate_offset(self, pose_map_tag):
        pose_map_offset = TransformStamped()

        matrix_map_tag= do_matrix(pose_map_tag)
        matrix_tag_Offset = do_Matrix_from_Pose(z=0.2)  #20cm Offset

        matrix_map_base = np.dot(matrix_map_tag, matrix_tag_Offset)

        mat_to_quat = tf3d.quaternion_from_matrix(matrix_map_base)      
        mat_to_trans = tf3d.translation_from_matrix(matrix_map_base)       
        print("matriix_map_base", matrix_map_base)
        print("matrix_to_quat", mat_to_quat)
        print("mat_to_trans", mat_to_trans)
        pose_map_offset.transform.translation.x = mat_to_trans[0]
        pose_map_offset.transform.translation.y = mat_to_trans[1]
        pose_map_offset.transform.translation.z = mat_to_trans[2]
        pose_map_offset.transform.rotation.x = mat_to_quat[1]
        pose_map_offset.transform.rotation.y = mat_to_quat[2]
        pose_map_offset.transform.rotation.z = mat_to_quat[3]
        pose_map_offset.transform.rotation.w = mat_to_quat[0]

        return pose_map_offset

    
    # #printet die komplette Liste der gefundenen Tags
    # def print_tag_list(self):

    #     #fischt sich alle INformationen aus Dictionary
    #     for i in self.Tag_Poses.keys():        
            
    #         #Dict Values in Liste abgewandelt
    #         abs_coordinateApril_value = list(self.Tag_Poses[i].values())

    #         print("----------------------------------------")
    #         print(" Name : " + str(i))
    #         print("Position: ")
    #         print("  X: " + str(abs_coordinateApril_value[0]))
    #         print("  Y: " + str(abs_coordinateApril_value[1]))
    #         # print("  Z: " + str(trans.transform.translation.z))
    #         # print("Orientation: ")
    #         # print("  X: " + str(trans.transform.rotation.x))
    #         # print("  Y: " + str(trans.transform.rotation.y))
    #         # print("  Z: " + str(trans.transform.rotation.z))
    #         # print("  W: " + str(trans.transform.rotation.w))


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
    # def broadcast_static_transform(self):
    #         print("1")
    #         self.map_tagpose = TransformStamped()

    #         self.map_tagpose.header.stamp = self.get_clock().now().to_msg()

    #         self.map_tagpose.header.frame_id = 'map'   #rechts unten
    #         print("dict", self.Tag_Poses)
    #         for i in self.Tag_Poses.keys():
    #             self.map_tagpose.child_frame_id = i    #Name Frame aus dictionary
    #             print("i", i)

    #             abs_coordinateApril_value = list(self.Tag_Poses[i].values())

    #             self.map_tagpose.transform.translation.x = abs_coordinateApril_value[0] #hier einzelne Werte aus dictionary rausgefischt
    #             self.map_tagpose.transform.translation.y = abs_coordinateApril_value[1]
    #             self.map_tagpose.transform.translation.z = abs_coordinateApril_value[2]
    #             self.map_tagpose.transform.rotation.x = abs_coordinateApril_value[4]
    #             self.map_tagpose.transform.rotation.y = abs_coordinateApril_value[5]
    #             self.map_tagpose.transform.rotation.z = abs_coordinateApril_value[6]
    #             self.map_tagpose.transform.rotation.w = abs_coordinateApril_value[3]

    #             self.tf_broadcaster.sendTransform(self.map_tagpose)   


def combine_transformations(trans_1, trans_2):
    pose_map_offset = TransformStamped()
    pose_map_offset.transform.translation.x = trans_1.transform.translation.x + trans_2.transform.translation.x 
    pose_map_offset.transform.translation.y = trans_1.transform.translation.y + trans_2.transform.translation.y
    pose_map_offset.transform.translation.z = trans_1.transform.translation.z + trans_2.transform.translation.z 

    x_1 = trans_1.transform.rotation.x
    y_1 = trans_1.transform.rotation.y
    z_1 = trans_1.transform.rotation.z
    w_1 = trans_1.transform.rotation.w

    x_2 = trans_2.transform.rotation.x
    y_2 = trans_2.transform.rotation.y
    z_2 = trans_2.transform.rotation.z
    w_2 = trans_2.transform.rotation.w

    pose_map_offset.transform.rotation.x = w_1 * x_2 + x_1 * w_2 + y_1 * z_2 - z_1 * y_2
    pose_map_offset.transform.rotation.y = w_1 * y_2 - x_1 * z_2 + y_1 * w_2 + z_1 * x_2
    pose_map_offset.transform.rotation.z = w_1 * z_2 + x_1 * y_2 - y_1 * x_2 + z_1 * w_2
    pose_map_offset.transform.rotation.w = w_1 * w_2 - x_1 * x_2 - y_1 * y_2 - z_1 * z_2
    
    return pose_map_offset

#Berechnung Transform zu einer Matrix
def do_matrix(input_m):
    transform = (input_m.transform.translation.x, input_m.transform.translation.y, input_m.transform.translation.z)
    rotation = (input_m.transform.rotation.w, input_m.transform.rotation.x, input_m.transform.rotation.y, input_m.transform.rotation.z)

    R = tf3d.quaternion_matrix(rotation)
    M = tf3d.translation_matrix(transform)


    K = tf3d.concatenate_matrices(M, R)
    return K


#Berechnung Pose zu einer Matrix
def do_Matrix_from_Pose(x = 0.0 , y = 0.0, z = 0.0, xo = 0.0, yo = 0.0, zo = 0.0, w = 1.0):
    transform = (x, y, z)
    rotation = (w, xo, yo, zo)

    R = tf3d.quaternion_matrix(rotation)
    M = tf3d.translation_matrix(transform)

    K = tf3d.concatenate_matrices(M, R)
    return K




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
