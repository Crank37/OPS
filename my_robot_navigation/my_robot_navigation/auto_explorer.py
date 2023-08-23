#!/usr/bin/env python3


from math import sqrt

import sys

import rclpy

from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus


import random


from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Bool 


# Map Bounds

map_min_x = -6.0

map_max_x = 3.0

map_min_y = -1.0

map_max_y = 8.0


success = True


def main():

  global auto_chaos

  global nav_to_pose_client

  global all_tags_detected
  all_tags_detected = False

  status = 4
  

  rclpy.init()


  auto_chaos = rclpy.create_node('auto_goals')


  # create Action Client object with desired message type and action name

  nav_to_pose_client = ActionClient(auto_chaos, NavigateToPose, 'navigate_to_pose')


  # wait for action server to come up

  while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):

    print("Server still not available; waiting...")

  subscription = auto_chaos.create_subscription(Bool, 'all_tags_detected', sub_callback, 10)


  while rclpy.ok():

    try:
      print(all_tags_detected)
      if all_tags_detected == False:
        
        position = generatePosition()

        orientation = generateOrientation()

        goal_handle = sendGoal(position, orientation)

        status = checkResult(goal_handle)

    except KeyboardInterrupt:

      print("Shutdown requested... complying...")

      break

    

  nav_to_pose_client.destroy()

  auto_chaos.destroy_node()

  rclpy.shutdown()

def sub_callback(msg):
  global all_tags_detected
  all_tags_detected = True
  return all_tags_detected
  


def sendGoal(position, orientation):

  """Create action goal object and send to action server, check if goal accepted"""

  global auto_chaos

  global nav_to_pose_client


  goal = NavigateToPose.Goal()

  goal.pose.header.frame_id = "map"


  goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()

  

  goal.pose.pose.position = position

  goal.pose.pose.orientation = orientation


  print("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))


  send_goal_future = nav_to_pose_client.send_goal_async(goal)

  rclpy.spin_until_future_complete(auto_chaos, send_goal_future)


  goal_handle = send_goal_future.result()


  if not goal_handle.accepted:

    print("Goal was rejected")

    nav_to_pose_client.destroy()

    auto_chaos.destroy_node()

    rclpy.shutdown()

    sys.exit(0)


  print("Goal Accepted!")


  return goal_handle


def checkResult(goal_handle):

  """Check for task completion while blocking further execution"""

  get_result_future = goal_handle.get_result_async()


  rclpy.spin_until_future_complete(auto_chaos, get_result_future)


  status = get_result_future.result().status
###############
  if all_tags_detected == True:
    status = GoalStatus.STATUS_SUCCEEDED
    ("Alle Tags erkannt")
    return status
  if status == GoalStatus.STATUS_SUCCEEDED:

    print("Reached Goal!!!")


  return status


def generatePosition():

  """Randomize a pair of values to denote xy position on map"""

  position = Point()

  position.x = round(random.uniform(map_min_x,map_max_x), 2)

  position.y = round(random.uniform(map_min_y,map_max_y), 2)

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


if __name__ == '__main__':

    main()
