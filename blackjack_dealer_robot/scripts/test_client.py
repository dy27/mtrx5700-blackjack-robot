#! /usr/bin/env python

import rospy
import actionlib
import blackjack_dealer_robot.msg
import geometry_msgs.msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys

def client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('dealer_action', blackjack_dealer_robot.msg.DealerAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    # goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    euler = euler_from_quaternion([-0.5, 0.5, 0.5, 0.5])
    print(euler)
    quat = quaternion_from_euler(*euler)
    print(quat)

    goal = blackjack_dealer_robot.msg.DealerGoal()
    goal.type = int(sys.argv[1])
    goal.pose = geometry_msgs.msg.Pose()
    goal.pose.position.x = 0.0
    goal.pose.position.y = 0.5
    goal.pose.position.z = 0.05
    goal.pose.orientation.x = -0.5
    goal.pose.orientation.y = 0.5
    goal.pose.orientation.z = 0.5
    goal.pose.orientation.w = 0.5

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    # print(result.result)
    print("RESULT RECEIVED", result)

    return result


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_client')
        result = client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
        # print("program interrupted before completion", file=sys.stderr)