from math import sqrt

import rclpy
from rclpy.node import Node 

import sys
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

import random


from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class autoExplore(Node):
    def __init__(self):
        super().__init__("auto_goals")

        #listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # create Action Client object with desired message type and action name
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') #Node richtig?

        #wait for action server to come up
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            print("Server still not available; waiting...")

        self.sucess = True
        self.status = 4

        #Eingrenzung Zufallswerte
        self.map_min_x = -1.0
        self.map_max_x = 3.0
        self.map_min_y = 0.0
        self.map_max_y = 4.0

        self.create_timer(0.5, self.send_goal)

    #Ziel ansteuern
    def send_goal(self):
        #Ziel in msg
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"    

        #x,y Position abfragen von zum anfahrenden TF
        self.check_pose()

        goal.pose.header.stamp = self.get_clock().now().to_msg() 
        try:
            goal.pose.pose.position.x = self.pos_x
            goal.pose.pose.position.y = self.pos_y
            
            print("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))

            #Vom CLient Ziel an Nav Server gesendet
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal) 
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                print("Goal was rejected")
                self.nav_to_pose_client.destroy()
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            print("Goal Accepted!")

            self.status = self.checkResult(goal_handle)
        except:
            print("fehler wegen lookup")


    def check_pose(self):
        from_frame = "map"
        to_frame = "Frame"

        try:
            
            trans = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            self.pos_x = trans.transform.translation.x
            print(self.pos_x)
            self.pos_y = trans.transform.translation.y
        except:
            print("lookup failed")     

        

    def checkResult(self, goal_handle):
        """Check for task completion while blocking further execution"""
        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self.nav_to_pose_client, get_result_future)
        status = get_result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Reached Goal!!!")
        return status


def main():
    rclpy.init()
    node = autoExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
