#! /usr/bin/env python

import rospy
import sys
import actionlib
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs    

from std_msgs.msg import Int32

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import blackjack_dealer_robot.msg

import geometry_msgs
from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import Pose

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

import copy

DEBUG_MODE = True

def DEBUG(message):
    if STEP_THROUGH:
        print(message)
        input("Press Enter to continue...")


class EndEffectorOffset:
    CARD_GRIP = [0.05, 0.1]
    CARD_SCOOP = [-0.05, 0.1]


class ActionType:
    DEAL = 0
    CLEAR = 1
    FLIP = 2
    MOVE = 3


class MotionPlanner:
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('motion_planner')

        # Set up transform listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get the transform from the table to base_link
        # self.transform = None
        # while self.transform is None:
        #     try:
        #         transform = tf_buffer.lookup_transform('base_link',
        #                                             'table', #source frame
        #                                             rospy.Time(0), #get the tf at first available time
        #                                             rospy.Duration(1.0)) #wait for 1 second
        #         self.transform = transform
        #     except:
        #         print("Failed")
        #         rospy.sleep(1)


        # Action server return message
        self.action_result = Int32()
        self.action_result.data = 1

        # Set up action server
        self.action_name = 'dealer_action'
        self.action_server = actionlib.SimpleActionServer(self.action_name,
                                                        blackjack_dealer_robot.msg.DealerAction,
                                                        execute_cb=self.execute_callback,
                                                        auto_start=False)
        self.action_server.start()



        self.shoe_position = [0.21, 0.085, 0.03]

        self.table_offset = [0.0, 1.2, 0.02]

        self.clearance = 0.1
        self.penetration = 0.005

        self.card_runway_y = 0.49

        self.card_shape = (0.064, 0.089)
        



        # First initialize moveit_commander and a rospy node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('build_tower_node', anonymous=True)

        # Instantiate a RobotCommander object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self.scene_interface = moveit_commander.PlanningSceneInterface()
        self.scene = moveit_commander.PlanningScene()

        # Instantiate a MoveGroupCommander object.
        # This object is an interface to a planning group (group of joints).
        # In our case the group consists of the primary joints in the UR5e robot, defined as "manipulator".
        # This interface can be used to plan and execute motions.
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        # Allow re planning to increase the odds of finding a solution
        self.move_group.allow_replanning(True)
        # Set the number of planning attempts - By default it is 1, we are increasing it to 10
        self.move_group.set_num_planning_attempts(10)
        # Set the amount of time allocated for generating a plan - By default it is 5s
        self.move_group.set_planning_time(20)

        # Misc variables
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.home_joint_angles = self.assign_home()

        rospy.spin()

    
    def table_to_robot_transform(table_point):
        table_x, table_y, table_z = table_point

        robot_x = table_x - self.table_offset[0]
        robot_y = -table_y - self.table_offset[1]
        robot_z = table_z - self.table_offset[2]

        return [robot_x, robot_y, robot_z]


    def execute_callback(self, goal):
        print("EXECUTE CALLBACK")
            
        if goal.type == ActionType.DEAL:
            self.deal_cards(goal.points, goal.use_runway_list, goal.flip_card_list)

        elif goal.type == ActionType.CLEAR:
            self.clear_cards()

        elif goal.type == ActionType.MOVE:
            self.move_eef_to_pose(goal.pose)

        elif goal.type == ActionType.FLIP:
            self.flip_card(goal.pose.position)

        self.action_server.set_succeeded(self.action_result)


    def deal_cards(self, points, use_runway_list):
        for point, use_runway, flip_card in zip(points, use_runway_list, flip_card_list):

            # Move to above shoe
            pose = geometry_msgs.msg.Pose()
            pose.position.x, pose.position.y, pose.position.z = self.table_to_robot_transform([self.shoe_position.x,
                                                                    self.shoe_position.y,
                                                                    self.shoe_position.z + self.clearance])
            eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
            move_eef_to_pose(eef_pose)

            # Move to shoe position to contact card
            pose.position.z -= self.clearance + self.penetration
            eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
            self.move_eef_to_pose(eef_pose)

            # Slide card out of shoe step 1
            pose.position.x += 0.02
            pose.position.z -= 0.0228
            eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
            self.move_eef_to_pose(eef_pose)

            # Slide card out of shoe step 2
            pose.position.x += 0.04
            pose.position.z -= 0.0072
            eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
            self.move_eef_to_pose(eef_pose)

            # Slide card out of shoe step 3
            pose.position.x += 0.05
            eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
            self.move_eef_to_pose(eef_pose)

            # Move card to runway
            pose.position.y = self.card_runway_y
            eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
            self.move_eef_to_pose(eef_pose)

            # Get target location to move card to
            target_point = self.table_to_robot_transform([point.x, point.y, point.z])

            if flip_card:
                # Move card to target location x plus allowance for flip
                pose.position.x = target_point.x - (self.card_shape[0] + 0.004)

                # Release grip on the card
                pose.position.z += self.clearance + self.penetration
                eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
                self.move_eef_to_pose(eef_pose)
            
                # Remember current pose
                current_pose = copy.deepcopy(pose)

                # Flip card
                flip_card(pose.position, table_ref_frame=False)

                # Move back to previous pose
                pose = current_pose
                eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
                self.move_eef_to_pose(eef_pose)

                # Regrip the card
                pose.z -= self.clearance + self.penetration
                eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
                self.move_eef_to_pose(eef_pose)

            else:
                # Move card to target location x
                pose.position.x = target_point.x
                eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=15)
                self.move_eef_to_pose(eef_pose)

            # Move card to target location y
            pose.position.y = target_point.y
            eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                            direction='left', angle_deg=15)
            self.move_eef_to_pose(eef_pose)

            # Release grip on the card
            pose.position.z += self.clearance + self.penetration
            eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                            direction='left', angle_deg=15)
            self.move_eef_to_pose(eef_pose)


    def clear_cards(self, points):
        pass


    def flip_card(point, table_ref_frame=True):
        if table_ref_frame:
            point.x, point.y, point.z = self.table_to_robot_transform([point.x, point.y, point.z])

        pose = geometry_msgs.msg.Pose()
        pose.position.x = point.x - (0.5 * self.card_shape[0] + 0.02)
        pose.position.y = point.y
        pose.position.z = point.z + self.clearance
        eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                direction='right', angle_deg=15)
        self.move_eef_to_pose(eef_pose)

        pose.position.z -= self.clearance + self.penetration

        pose.
        eef_pose = self.eef_pose_transform(pose, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                direction='right', angle_deg=15)
        self.move_eef_to_pose(eef_pose)



    def eef_pose_transform(self, target_pose, end_effector_offset, direction='left', angle_deg=15):
        """Returns the pose to move the robot arm to such that the end effector is at target_pose."""

        assert 0 <= angle_deg <= 90
        angle_rad = math.radians(angle_deg)

        if direction == 'left':
            euler = [math.pi/2, math.pi/2 + angle_rad, 0]
        elif direction == 'right':
            euler = [math.pi/2, math.pi/2 - angle_rad, 0]
        else:
            raise Exception('Invalid direction specified.')

        old_target_position = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
        # old_target_quaternion = [target_pose.orientation.x, target_pose.orientation.y,
        #                           target_pose.orientation.z, target_pose.orientation.w]

        
        cos_angle = math.cos(angle_rad)
        sin_angle = math.sin(angle_rad)

        x, y = end_effector_offset

        negative_x = False
        if x < 0:
            negative_x = True
            x = abs(x)

        if not negative_x:
            target_pose.position.x += -x*cos_angle + y*sin_angle
        else:
            target_pose.position.x -= -x*cos_angle + y*sin_angle

        target_pose.position.z += x*sin_angle + y*cos_angle

        target_quat = quaternion_from_euler(*euler)
        target_pose.orientation.x = target_quat[0]
        target_pose.orientation.y = target_quat[1]
        target_pose.orientation.z = target_quat[2]
        target_pose.orientation.w = target_quat[3]

        return target_pose


    def move_to_pose(self, table_pose):

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, self.transform)


    def move_eef_to_pose(self, pose_goal):
        """
        Moves the end-effector of the robot to a desired pose. Note that the entire robot moves to achieve.

        :param pose_goal: The desired pose to which the end effector of the robot has to move to.
        :type pose_goal: geometry_msgs.msg.Pose
        """
        # Set the desired goal
        self.move_group.set_pose_target(pose_goal)
        # Move the end effector to the desired pose.
        self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()


    def plan_cartesian_path(self, waypoints, eef_step=0.01, jump=0.0, max_attempts=200):
        """
        Plans a Cartesian path for the end-effector of the robot following the specified waypoints.

        This function only plans the path (if it exists).
        Use execute_plan() to execute the planned path and display_trajectory() to display the planed path
        """
        fraction = 0.0
        attempts = 0
        while fraction < 1.0 and attempts < max_attempts:
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step, jump, True)
            # Increment the number of attempts
            attempts += 1

            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                return plan, fraction
            else:
                rospy.loginfo("Path planning failed with only " +
                              str(fraction) + " success after " +
                              str(max_attempts) + " attempts.")
                return plan, fraction


    def execute_plan(self, plan):
        """
        Executes a plan and moves the robot such that the it's end effector follows the specified plan

        :param plan: The plan to execute. Use plan_cartesian_path to plan the path
        :type plan: List
        """
        self.move_group.execute(plan, wait=True)
        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


if __name__ == '__main__':
    MotionPlanner()