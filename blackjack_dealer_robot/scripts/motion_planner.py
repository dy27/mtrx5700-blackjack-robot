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

DEBUG_MODE = False

def DEBUG(message):
    if DEBUG_MODE:
        print(message)
        raw_input("Press Enter to continue...")


class EndEffectorOffset:
    CARD_GRIP = [-0.055, -0.024]
    CARD_SCOOP = [0.055, -0.024]
    # CARD_GRIP = [0., -0.024]
    # CARD_SCOOP = [0., -0.024]

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



        self.shoe_position = [0.208, 0.54, 0.03]

        self.table_offset = [-0.525, 0.675, 0.012]
        # self.table_offset = [-0.45, 0.9, 0.0]

        self.table_shape = (0.9, 0.6)

        self.clearance = 0.05
        self.penetration = 0.0075

        self.card_runway_y = 0.29

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

    
    def table_to_robot_transform(self, table_point):
        table_x, table_y, table_z = table_point

        robot_x = table_x + self.table_offset[0]
        robot_y = -table_y + self.table_offset[1]
        robot_z = table_z + self.table_offset[2]

        return [robot_x, robot_y, robot_z]


    def execute_callback(self, goal):
        print("EXECUTE CALLBACK")
            
        if goal.type == ActionType.DEAL:
            self.deal_cards(goal.card_points, goal.use_runway_list, goal.flip_card_list)

        elif goal.type == ActionType.CLEAR:
            self.clear_cards(goal.card_points)

        elif goal.type == ActionType.MOVE:
            self.move_eef_to_pose(goal.pose)

        elif goal.type == ActionType.FLIP:
            for p in goal.card_points:
                self.flip_card([p.x, p.y, p.z])

        self.action_server.set_succeeded(self.action_result)


    def deal_cards(self, points, use_runway_list, flip_card_list):

        DEBUG('starting dealing')

        pose_waypoints = []

        for point, use_runway, flip in zip(points, use_runway_list, flip_card_list):

            # Move to above shoe
            table_point = [self.shoe_position[0], self.shoe_position[1], self.shoe_position[2] + self.clearance]
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                    direction='left', angle_deg=0, linear=True)
            DEBUG('starting dealing')

            # Contact card
            self.penetration += 0.0025
            table_point[2] -= self.clearance + self.penetration
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                    direction='left', angle_deg=0, linear=True)
            DEBUG('starting dealing')

            # Extraction step 1
            table_point[0] += 0.02
            table_point[2] -= 0.0228
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                    direction='left', angle_deg=15, linear=True)
            DEBUG('starting dealing')

            # Extraction step 2
            self.penetration -= 0.0025
            table_point[0] += 0.04
            table_point[2] -= 0.0072
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                    direction='left', angle_deg=30, linear=True)
            DEBUG('starting dealing')

            # Extraction step 3
            table_point[0] += 0.05
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                    direction='left', angle_deg=30, linear=True)
            DEBUG('starting dealing')


            table_point[1] = self.card_runway_y
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                    direction='left', angle_deg=30, linear=True)
            DEBUG('starting dealing')

            if flip:
                card_shift = 0.000

                # Don't move card into place if the target position is too close to the left edge
                if point.x - (self.card_shape[0] + card_shift) >= 0.05:
                    # Move to target x value plus allowance for flip
                    table_point[0] = point.x - (self.card_shape[0] + card_shift)
                    self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                            direction='left', angle_deg=15, linear=True)
                    DEBUG('starting dealing')

                # Flip card
                table_point = self.flip_card(table_point, card_shift=card_shift)

                # Move above new card location
                table_point[2] = self.clearance
                self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                        direction='left', angle_deg=30, linear=True)
                DEBUG('starting dealing')

                # Regrip card
                table_point[2] = -self.penetration
                self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                        direction='left', angle_deg=30, linear=True)
                DEBUG('starting dealing')

                # Adjust card position if necessary
                if table_point[0] != point.x:
                    table_point[0] = point.x
                    self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                            direction='left', angle_deg=30, linear=True)
                    DEBUG('starting dealing')

            else:
                # Move to target x value
                table_point[0] = point.x
                self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                        direction='left', angle_deg=30, linear=True)
                DEBUG('starting dealing')

            # Move to target y value
            table_point[1] = point.y
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                    direction='left', angle_deg=30, linear=True)
            DEBUG('starting dealing')

            # Move up 
            table_point[2] = self.clearance
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                    direction='left', angle_deg=30, linear=True)
            DEBUG('starting dealing')


    def clear_cards(self, points):
        if len(points) == 0:
            return

        DEBUG('starting dealing')
        table_point = [points[0].x, points[0].y, self.clearance]
        table_point[0] -= self.card_shape[0]/2 + 0.02
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                    direction='right', angle_deg=15, linear=True)

        DEBUG('starting dealing')
        table_point[2] = -self.penetration
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                    direction='right', angle_deg=15, linear=True)

        DEBUG('starting dealing')
        for point in points:
            table_point[0] = point.x
            table_point[1] = point.y
            table_point[2] = -self.penetration
            self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                    direction='right', angle_deg=15, linear=True)

        DEBUG('starting dealing')
        table_point[0] = self.table_shape[0] + 0.01
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                    direction='right', angle_deg=15, linear=True)

        DEBUG('starting dealing')
        table_point[2] = self.clearance
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                    direction='right', angle_deg=15, linear=True)

        DEBUG('starting dealing')
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                    direction='right', angle_deg=35, linear=True)


    def flip_card(self, table_point, restore_card_position=False, allowance=0.02, card_leverage=0.02, card_shift=0.005):
        """Flip a card located at table_point"""
        # # Move to target x value plus allowance for flip
        # table_point[0] = point.x - (self.card_shape[0]/2 + 0.002)
        # self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
        #                                         direction='right', angle_deg=15, linear=True)
        # DEBUG('starting dealing')

        DEBUG('BEGINNING FLIP')

        # Move up to clearance
        table_point[2] = self.clearance
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                                                direction='left', angle_deg=30, linear=True)
        DEBUG('starting dealing')

        # Store copy of original point
        original_point = copy.deepcopy(table_point)

        # Move to allow for card scoop
        table_point[0] -= self.card_shape[0]/2 + allowance
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                direction='right', angle_deg=15, linear=True)
        DEBUG('starting dealing')

        # Move down
        table_point[2] = -self.penetration
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                direction='right', angle_deg=15, linear=True)
        DEBUG('starting dealing')

        # Move under card
        table_point[0] += allowance + card_leverage
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                direction='right', angle_deg=15, linear=True)
        DEBUG('starting dealing')

        # Lift card partially up
        table_point[2] += 0.025
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                direction='right', angle_deg=15, linear=True)
        DEBUG('starting dealing')

        # Move up and across to flip
        table_point[0] += 0.1
        table_point[2] += 0.03
        self.move_to_table_point(table_point, end_effector_offset=EndEffectorOffset.CARD_SCOOP,
                                                direction='right', angle_deg=15, linear=True)
        DEBUG('starting dealing')

        new_table_point = original_point
        new_table_point[0] += self.card_shape[0] + card_shift

        return new_table_point



        






    def eef_pose_transform(self, target_pose, end_effector_offset, direction='left', angle_deg=15):
        """Returns the pose to move the robot arm to such that the end effector is at target_pose."""

        # angle_deg = 0

        assert 0 <= angle_deg <= 90
        angle_rad = math.radians(angle_deg)

        if direction == 'left':
            euler = [math.pi/2, math.pi/2 - angle_rad, 0]
        elif direction == 'right':
            euler = [math.pi/2, math.pi/2 + angle_rad, 0]
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

        transformed_pose = geometry_msgs.msg.Pose()

        if not negative_x:
            transformed_pose.position.x = target_pose.position.x - x*cos_angle + y*sin_angle
        else:
            transformed_pose.position.x = target_pose.position.x + x*cos_angle - y*sin_angle

        transformed_pose.position.y = target_pose.position.y + 0.027
        transformed_pose.position.z = target_pose.position.z + x*sin_angle + y*cos_angle

        target_quat = quaternion_from_euler(*euler)
        print(target_quat)
        transformed_pose.orientation.x = target_quat[0]
        transformed_pose.orientation.y = target_quat[1]
        transformed_pose.orientation.z = target_quat[2]
        transformed_pose.orientation.w = target_quat[3]
        print(transformed_pose)

        return transformed_pose


    def move_to_table_point(self, table_point, end_effector_offset=EndEffectorOffset.CARD_GRIP,
                            direction='left', angle_deg=15, linear=False):
        pose = geometry_msgs.msg.Pose()
        pose.position.x, pose.position.y, pose.position.z = self.table_to_robot_transform(table_point)
        eef_pose = self.eef_pose_transform(pose, end_effector_offset, direction, angle_deg)
        if linear:
            plan, _ = self.plan_cartesian_path([eef_pose])
            self.execute_plan(plan)
        else:
            self.move_eef_to_pose(eef_pose)


    def move_linear(self, current_pose, target_pose):
        waypoints = [current_pose, target_pose]
        plan, _ = self.plan_cartesian_path(waypoints)
        self.execute_plan(plan)


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


    # def linear_interpolation()


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