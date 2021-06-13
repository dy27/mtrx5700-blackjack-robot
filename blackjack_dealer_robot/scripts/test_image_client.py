#! /usr/bin/env python

import rospy
import actionlib
import blackjack_dealer_robot.msg
import geometry_msgs.msg
from blackjack_dealer_robot.msg import DetectCardsAction
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys

def client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('card_detection', blackjack_dealer_robot.msg.DetectCardsAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    # goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # euler = euler_from_quaternion([-0.5, 0.5, 0.5, 0.5])
    # print(euler)
    # quat = quaternion_from_euler(*euler)
    # print(quat)

    goal = blackjack_dealer_robot.msg.DetectCardsGoal()
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print("RESULT RECEIVED", result)

    

    # full_round(client)

    # clear(client)

    return

    goal = blackjack_dealer_robot.msg.DealerGoal()
    goal.type = 0
    goal.card_points = []
    point = geometry_msgs.msg.Point()
    point.x, point.y, point.z = 0.85, 0.105, 0.0
    goal.card_points.append(point)

    goal.use_runway_list = [True]
    goal.flip_card_list = [True]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    # print(result.result)
    print("RESULT RECEIVED", result)

    return result

def clear(client):
    goal = blackjack_dealer_robot.msg.DealerGoal()
    goal.type = 1

    clear_points = [[0.21, 0.21, 0]]
    for x, y, z in clear_points:
        point_msg = geometry_msgs.msg.Point()
        point_msg.x, point_msg.y, point_msg.z = x, y, z
        goal.card_points.append(point_msg)

    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print("RESULT RECEIVED", result)


def full_round(client):
    # Initial deal
    goal = blackjack_dealer_robot.msg.DealerGoal()
    goal.type = 0
    points = [[0.225, 0.065, 0], [0.45, 0.065, 0], [0.69, 0.065, 0], [0.6, 0.4, 0], [0.305, 0.065, 0], [0.53, 0.065, 0], [0.77, 0.065, 0], [0.46, 0.4, 0]]
    runway = [True, True, True, True, True, True, True, True]
    flip = [True, True, True, True, True, True, True, False]
    for p, r, f in zip(points, runway, flip):
        point_msg = geometry_msgs.msg.Point()
        point_msg.x, point_msg.y, point_msg.z = p[0], p[1], p[2]
        goal.card_points.append(point_msg)
        goal.use_runway_list.append(r)
        goal.flip_card_list.append(f)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print("RESULT RECEIVED", result)

    # Flip dealer card
    goal = blackjack_dealer_robot.msg.DealerGoal()
    goal.type = 2
    points = [[0.46, 0.4, 0]]
    for p in points:
        point_msg = geometry_msgs.msg.Point()
        point_msg.x, point_msg.y, point_msg.z = p[0], p[1], p[2]
        goal.card_points.append(point_msg)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print("RESULT RECEIVED", result)

    # Clear 1
    goal = blackjack_dealer_robot.msg.DealerGoal()
    goal.type = 1
    points = [[0.225, 0.065, 0]]
    for p in points:
        point_msg = geometry_msgs.msg.Point()
        point_msg.x, point_msg.y, point_msg.z = p[0], p[1], p[2]
        goal.card_points.append(point_msg)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print("RESULT RECEIVED", result)

    # Clear 2
    goal = blackjack_dealer_robot.msg.DealerGoal()
    goal.type = 1
    points = [[0.46+0.06, 0.4, 0]]
    for p in points:
        point_msg = geometry_msgs.msg.Point()
        point_msg.x, point_msg.y, point_msg.z = p[0], p[1], p[2]
        goal.card_points.append(point_msg)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print("RESULT RECEIVED", result)


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