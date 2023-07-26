import rclpy
from rclpy.node import Node 

from geometry_msgs.msg import TransformStamped


from tf2_ros import TransformBroadcaster

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class broadcasting(Node):
    def __init__(self):
        super().__init__("frame_broadcaster")
      

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.broadcast_static_transform)      
          
        #gemappte Tags
        self.tag_id_coordinates = {
                                    "Frame" : {"x_t" : 3.0, "y_t" : 3.0, "z_t" : 0. , "w_o" : 1.0, "x_o" : 0.0, "y_o" : 0., "z_o" : 0.0},
                                    }



    #Alle Fake Tags relativ zur Map werden gepublished für die bessere Visualisierung
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
    node = broadcasting()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
