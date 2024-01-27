#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs
import tf
from math import pi
from control_msgs.msg import GripperCommandActionGoal

## Function to get the joint position of a certain configuration
def pose_defined(name):
    group.set_named_target(name)
    plan = group.plan()
    group.go()
   
##Get the current position of the robot, will be use to modify the current position
def current_pose():
    c_pose = geometry_msgs.msg.Pose()
    current=group.get_current_pose().pose
    c_pose.orientation.x=current.orientation.x
    c_pose.orientation.y=current.orientation.y
    c_pose.orientation.z=current.orientation.z
    c_pose.orientation.w=current.orientation.w
    c_pose.position.x = current.position.x
    c_pose.position.y = current.position.y
    c_pose.position.z = current.position.z
    return c_pose

##Get the color
def color_callback(data):
    color=data
   
##Defines the variables to arranges in array form the bottle positions, the amount of distances between each bottle and the value to publish the gripper position value
gripper_open = 0.0
gripper_close = 0.152
i_r_x=0.0
i_r_y=0.0
i_g_x=0.0
i_g_y=0.0
i_o_x=0.0
i_o_y=0.0
paso=0.105

color=""

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("gp4_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
gripper_pub = rospy.Publisher('/gripper/gripper_cmd/goal', GripperCommandActionGoal, queue_size=1)
color_sub = rospy.Subscriber('/current_color', String, color_callback)

gripper_comm_close= GripperCommandActionGoal()
gripper_comm_open= GripperCommandActionGoal()
pose_target = geometry_msgs.msg.Pose()

gripper_comm_close.goal.command.position = gripper_close
gripper_comm_open.goal.command.position = gripper_open

while not rospy.is_shutdown():

    waypoints = []

    ##There is 1 if for each color option
    if(color == "red"):
    	## Get the position that hovers over the pick position
    	pose_defined("pick_pose")
    	rospy.sleep(2)
	
	## Lower the end effector position
    	pose_target=current_pose()
    	pose_target.position.z = pose_target.position.z - 0.215


    	waypoints.append(copy.deepcopy(pose_target))


    	fraction = 0.0

    	(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold


    	group.execute(plan, wait=True)

    	rospy.sleep(1)

    	group.stop()
    	group.clear_pose_targets()
    	waypoints = []
	
	## Close the Gripper
    	gripper_pub.publish(gripper_comm_close)

    	rospy.sleep(2)
	
	##Rise the end effector and move the bottle to the designated area
    	pose_defined("pick_pose")
    	rospy.sleep(1)

    	pose_defined("place_r")
    	rospy.sleep(1)
    	
	## If it is the first bottle, it will put the bottle on the original position, otherwise it will modify the x and y position
    	if(i_r_x!=0 or i_r_y!=0):
        	pose_target=current_pose()
        	pose_target.position.x = pose_target.position.x - paso*i_r_x
        	pose_target.position.y = pose_target.position.y + paso*i_r_y
        	waypoints.append(copy.deepcopy(pose_target))

        	fraction = 0.0

        	(plan, fraction) = group.compute_cartesian_path(
               	                    waypoints,   # waypoints to follow
               	                    0.01,        # eef_step
               	                    0.0)         # jump_threshold

        	group.execute(plan, wait=True)

        	rospy.sleep(1)
        	group.stop()
        	group.clear_pose_targets()
        	waypoints = []

	## Opens the gripper to release the bottle
    	gripper_pub.publish(gripper_comm_open)

    	rospy.sleep(3)
	
	##Return the manipulator to the original position
    	pose_defined("home")

	##Save that a bottle of this color have been stored and will modify the next position of the bottle with the same color
    	i_r_y=i_r_y + 1.0
    	
    	##This defined how many bottles will be position next to each other before moving to the next row
    	if(i_r_y>1):
        	i_r_y=0.0
        	i_r_x=i_r_x + 1.0

    	rospy.sleep(5)

    if(color == "green"):
    	pose_defined("pick_pose")
    	rospy.sleep(2)

    	pose_target=current_pose()
    	pose_target.position.z = pose_target.position.z - 0.215


    	waypoints.append(copy.deepcopy(pose_target))


    	fraction = 0.0

    	(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold


    	group.execute(plan, wait=True)

    	rospy.sleep(1)

    	group.stop()
    	group.clear_pose_targets()
    	waypoints = []

    	gripper_pub.publish(gripper_comm_close)

    	rospy.sleep(2)

    	pose_defined("pick_pose")
    	rospy.sleep(1)

    	pose_defined("place_g")
    	rospy.sleep(1)

    	if(i_g_x!=0 or i_g_y!=0):
        	pose_target=current_pose()
        	pose_target.position.x = pose_target.position.x + paso*i_g_x
        	pose_target.position.y = pose_target.position.y + paso*i_g_y
        	waypoints.append(copy.deepcopy(pose_target))

        	fraction = 0.0

        	(plan, fraction) = group.compute_cartesian_path(
               	                    waypoints,   # waypoints to follow
               	                    0.01,        # eef_step
               	                    0.0)         # jump_threshold

        	group.execute(plan, wait=True)

        	rospy.sleep(1)
        	group.stop()
        	group.clear_pose_targets()
        	waypoints = []


    	gripper_pub.publish(gripper_comm_open)

    	rospy.sleep(3)

    	pose_defined("home")

    	i_g_y=i_g_y + 1.0
    	if(i_g_y>1):
        	i_g_y=0.0
        	i_g_x=i_g_x + 1.0

    	rospy.sleep(5)

	if(color == "orange"):
    	pose_defined("pick_pose")
    	rospy.sleep(2)

    	pose_target=current_pose()
    	pose_target.position.z = pose_target.position.z - 0.215


    	waypoints.append(copy.deepcopy(pose_target))


    	fraction = 0.0

    	(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold


    	group.execute(plan, wait=True)

    	rospy.sleep(1)

    	group.stop()
    	group.clear_pose_targets()
    	waypoints = []

    	gripper_pub.publish(gripper_comm_close)

    	rospy.sleep(2)

    	pose_defined("pick_pose")
    	rospy.sleep(1)

    	pose_defined("place_o")
    	rospy.sleep(1)

    	if(i_o_x!=0 or i_o_y!=0):
        	pose_target=current_pose()
        	pose_target.position.x = pose_target.position.x - paso*i_o_x
        	pose_target.position.y = pose_target.position.y + paso*i_o_y
        	waypoints.append(copy.deepcopy(pose_target))

        	fraction = 0.0

        	(plan, fraction) = group.compute_cartesian_path(
               	                    waypoints,   # waypoints to follow
               	                    0.01,        # eef_step
               	                    0.0)         # jump_threshold

        	group.execute(plan, wait=True)

        	rospy.sleep(1)
        	group.stop()
        	group.clear_pose_targets()
        	waypoints = []


    	gripper_pub.publish(gripper_comm_open)

    	rospy.sleep(3)

    	pose_defined("home")

    	i_o_y=i_o_y + 1.0
    	if(i_o_x>1):
        	i_o_x=0.0
        	i_o_y=i_o_y + 1.0

    	rospy.sleep(5)


	color=""


moveit_commander.roscpp_shutdown()
