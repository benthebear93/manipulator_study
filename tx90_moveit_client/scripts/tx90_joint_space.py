#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

def pose_target_gen(pos):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = pos[0]
    pose_target.orientation.x = pos[1]
    pose_target.orientation.y = pos[2]
    pose_target.orientation.z = pos[3]
    pose_target.position.x = pos[4]
    pose_target.position.y = pos[5]
    pose_target.position.z = pos[6]
    return pose_target

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('staubli_control', anonymous=False)

    # Get instance from moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Get group_commander from MoveGroupCommander
    group_name = "tx_90"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    '''display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)'''
    # Move using joint space

    pos_dic = {
    'init_pose':[1.0, 0.0, 0.0, 0.0, 0.05, 0.05 ,1.428],
    'cfrp_pos1':[0.5, -0.5, 0.5, -0.5, 0.7, 0.2, 1],
    }
    #cfrp_pos1 = [0.7, 0.7, 0.2, 1]
    pose_target = pose_target_gen(pos_dic['cfrp_pos1'])
    move_group.set_pose_target(pose_target)
    plan1 = move_group.plan()
    move_group.go(wait=True)
    move_group.stop()

    #default = [1, 0.05, 0.05, 1.428]
    pose_target = pose_target_gen(pos_dic['init_pose'])
    move_group.set_pose_target(pose_target)
    plan1 = move_group.plan()

    move_group.go(wait=True)
    move_group.stop()
    # Move using joint space
    # joint_goal = move_group.get_current_joint_values()
    # print joint_goal

    # joint_goal[0] = 0
    # joint_goal[1] = 0
    # joint_goal[2] = 0
    # joint_goal[3] = 0
    # joint_goal[4] = 0
    # joint_goal[5] = 0

    current_joints = move_group.get_current_joint_values()
    print current_joints

    quit()

if __name__ == "__main__":
    main()