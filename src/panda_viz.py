#!/usr/bin/python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import numpy as np
from tools.get_state_validity import StateValidity

class PandaViz:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_viz')

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planner_id('RRTstarkConfigDefault')
        self.group.set_planning_time(3)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)
        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Robot Groups:", self.robot.get_group_names())
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""
        joint_vals=self.group.get_random_joint_values()
        print("============ bounds:%s" % joint_vals)
        self.add_shelf()

        self.sv = StateValidity()

    def add_box(self,pos,size,name="box",timeout=4):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = pos[0]
        box_pose.pose.position.y = pos[1]
        box_pose.pose.position.z = pos[2]
        box_name = name
        self.scene.add_box(box_name, box_pose, size=size)

    def add_shelf(self):
        x=0.65
        pos=[[x, 0, 0.20], [x, 0, 0.60], [x, 0, 1.0], [x, 0, 1.4], [x, 0.55, .7], [x, -0.55, .7]]
        size=[[0.60, 1.1, 0.02],[0.60, 1.1, 0.02], [0.60, 1.1, 0.02], [0.60, 1.1, 0.02], [0.60, 0.02, 1.4], [0.60, 0.02, 1.4]]
        for i in range(len(pos)):
            self.add_box(pos[i],size[i],name="box"+str(i))

    def go_to_joint_state(self,start=None,goal=None):
        if(start==None):
            start=self.group.get_random_joint_values()
            start.append(0.035)
            start.append(0.035)
            joint_state = JointState()
            joint_state.header.frame_id=self.robot.get_current_state().joint_state.header.frame_id
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = self.robot.get_current_state().joint_state.name
            joint_state.position = start
            moveit_robot_state = RobotState()
            moveit_robot_state.joint_state = joint_state
            self.group.set_start_state(moveit_robot_state)
        if(goal==None):
            goal=self.group.get_random_joint_values()
        self.group.go(goal, wait=True)
        self.group.stop()

    def random_invalid_state(self):
        for i in range(10):
            start=self.group.get_random_joint_values()
            start.append(0.035)
            start.append(0.035)
            joint_state = JointState()
            joint_state.header.frame_id=self.robot.get_current_state().joint_state.header.frame_id
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = self.robot.get_current_state().joint_state.name
            joint_state.position = start
            moveit_robot_state = RobotState()
            moveit_robot_state.joint_state = joint_state
            if( not self.sv.getStateValidity(moveit_robot_state, group_name="panda_arm")):
                print("found invalid in:",i)
                self.group.set_start_state(moveit_robot_state)
                break
        goal=self.group.get_random_joint_values()
        # print("current joint:",self.group.get_current_joint_values())
        # self.group.go(start[:7], wait=True)
        # self.group.stop()
        print("current joint:",self.group.get_current_joint_values())
        plan=self.group.plan(goal)
        # self.group.execute(plan)
        # print("current joint:",self.group.get_current_joint_values())
        self.group.stop()


if __name__ == '__main__':
    pv=PandaViz()
    # pv.go_to_joint_state()
    pv.random_invalid_state()
