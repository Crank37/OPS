import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Publisher and subscriber turning a Joy message into a Geometry twist message compatible with the Leo Robot

def main():
    global pub
    rclpy.init()
    joy_handel = rclpy.create_node('joy_convert')
    joy_handel.create_subscription(Joy, 'joy', mysubcallback, 10)
    pub = joy_handel.create_publisher(Twist, '/cmd_vel', 1)
    try:
        rclpy.spin(joy_handel)
    except KeyboardInterrupt:
        pass
    
    joy_handel.destroy_node()
    rclpy.shutdown()




def mysubcallback(msg):
    # convert Joy into twist and publish
    global pub
    twistcommand = Twist()
    twistcommand.linear.x = msg.axes[1]
    #twistcommand.linear.y = msg.axes[0]
    #twistcommand.linear.z = msg
    #twistcommand.angular.x =
    #twistcommand.angular.y =
    twistcommand.angular.z = msg.axes[0]
    pub.publish(twistcommand)




if __name__ == '__main__':
    main()

