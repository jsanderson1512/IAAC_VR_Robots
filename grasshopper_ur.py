#!/usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
import signal
import sys
import os
import numpy as np
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint
from rospy.impl.transport import DeadTransport
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL
import math
from geometry_msgs.msg import Pose, Twist


a = []


# def callback(data):
#     a = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
#     print (a)



def listener():
    rospy.Subscriber("/target_plane", Twist, callback)
    rospy.spin()


def callback(data):

    a = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
    print (a)
    # Random 2-point trajectory
    duration = 10
    x,y,z, ax, ay, az = a
    v = PyKDL.Vector(x,y,z)
    # create a rotation from XYZ Euler angles
    r2 = PyKDL.Rotation.EulerZYX(ax, ay, az)
    # create a frame from a vector and a rotation
    f = PyKDL.Frame(r2, v)
    q = f.M.GetQuaternion()

    p = CartesianTrajectoryPoint()
    p.pose.position.x = f.p[0]
    p.pose.position.y = f.p[1]
    p.pose.position.z = f.p[2]
    q = PyKDL.Rotation.GetQuaternion(f.M)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]

    print("origin: ", f.p)
    print("quaternion: ", q)

    #p1 = random_point()
    #p2 = random_point()
    
    p.time_from_start = rospy.Duration(0.5 * duration)
    
    #p2.time_from_start = rospy.Duration(duration)

    goal = FollowCartesianTrajectoryGoal()
    goal.trajectory.points.append(p)
    
    #goal.trajectory.points.append(p2)

    client.send_goal(goal)
    client.wait_for_result()


if __name__ == "__main__" :
    try:    
        rospy.init_node('action_test_client')
        client = actionlib.SimpleActionClient(
            '/pose_based_cartesian_traj_controller/follow_cartesian_trajectory',
            FollowCartesianTrajectoryAction)
        client.wait_for_server()

        # Suppress spam output of urdf parsing.
        # urdf_parser_py is unhappy with various visual tags in the robot_description.
        tmp = sys.stderr
        sys.stderr = open(os.devnull, 'w')
        robot = URDF.from_parameter_server()
        sys.stderr = tmp

        _, tree = treeFromUrdfModel(robot)
        fk_solver = PyKDL.ChainFkSolverPos_recursive(tree.getChain('base_link', 'tool0'))
        listener()

    except rospy.ROSInterruptException:
         pass