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
from std_msgs.msg import Bool #oder BOOL, weiß nicht

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class autoExplore(Node):
    def __init__(self):
        super().__init__("auto_explore")


        # create Action Client object with desired message type and action name 
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') #Node richtig?

        #mittels Terminal, um zu abfahrenden Tag zuzuweisen (0 initialzustand, 9-10 sind die Tags in der Liste nach reihenfolge)
        # $ ros2 topic pub --once   /choose_tag std_msgs/String "data: 'name'" 
        self.limit = self.create_subscription(Bool, "/maxcount", self.max_count_cb, 1)

        #wait for action server to come up
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            print("Server still not available; waiting...")

        self.status = False #True falls alle tags gefunden

        #Eingrenzung Zufallswerte
        self.map_min_x = -1.0
        self.map_max_x = 3.0
        self.map_min_y = 0.0
        self.map_max_y = 4.0

        self.create_timer(0.5, self.loop)

    def loop(self):
        if not self.status:
            position = self.generatePosition()
            orientation = generateOrientation()

            goal_handle = self.send_goal(position, orientation)

            status = self.checkResult(goal_handle)

    #Ziel ansteuern
    def send_goal(self, position, orientation):
        #Ziel in msg
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"    

        #x,y Position abfragen von zum anfahrenden TF

        goal.pose.header.stamp = self.get_clock().now().to_msg() 
        try:
            goal.pose.pose.position = position
            goal.pose.pose.orientation = orientation

            print("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))

            #Vom CLient Ziel an Nav Server gesendet
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal) 
            # rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            #kommt hier nicht weiter????
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                print("Goal was rejected")
                self.nav_to_pose_client.destroy()
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            print("Goal Accepted!")

            return goal_handle
        except:
            print("fehler wegen lookup")


    def max_count_cb(self, data):
        if data:
            self.status = True



    def checkResult(self, goal_handle):
        """Check for task completion while blocking further execution"""
        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self.nav_to_pose_client, get_result_future)
        status = get_result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Reached Goal!!!")
        return status

    def generatePosition(self):
        """Randomize a pair of values to denote xy position on map"""
        position = Point()
        position.x = round(random.uniform(self.map_min_x,self.map_max_x), 2)
        position.y = round(random.uniform(self.map_min_y,self.map_max_y), 2)
        position.z = 0.0
        return position

def generateOrientation():
  """Randomize a pair of values to denote yaw orientation on map"""
  quat = Quaternion()
  quat.w = round(random.uniform(-1.0,1.0),3)
  quat.x = 0.0
  quat.y = 0.0
  quat.z = sqrt(1 - quat.w*quat.w)
  return quat


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


