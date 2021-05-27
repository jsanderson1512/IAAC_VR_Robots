#!/usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
import signal
import sys
import os
import tf
import numpy as np
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint
from rospy.impl.transport import DeadTransport
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL
import math
from geometry_msgs.msg import Pose, Twist, TransformStamped
from std_msgs.msg import String, Float64, Int32MultiArray
from ur_msgs.srv import SetIO 


def listener(tf_listener):
    rospy.Subscriber("/target_speed", Float64, speedCallback)
    rospy.Subscriber("/target_IO", Int32MultiArray, ioCallback)
    rospy.Subscriber("/target_plane_twist", Twist, poseCallback)
    rospy.Subscriber("/tf", TransformStamped, tfCallback)
    rospy.spin()


def tfCallback(msg):
    global current_pose
    for trans in msg.transforms:
        if trans.child_frame_id == "tool0_controller":
            c_position = trans.transform.translation
            current_pose = [c_position.x , c_position.y, c_position.z]


def speedCallback(msg):
    global speed
    
    speed = msg.data
    print(speed, type(speed))


def ioCallback(msg):
    global speed
    
    rospy.wait_for_service("/ur_hardware_interface/set_io")
    set_io = rospy.ServiceProxy("/ur_hardware_interface/set_io", SetIO)
    set_io(fun = 1, pin = msg.data[0], state = msg.data[1])
    print(speed)


def computeDuration(start_pose, end_pose, speed):
    start_pose = np.array(start_pose)
    end_pose = np.array(end_pose[:3])
    distance = np.linalg.norm(end_pose - start_pose) * 1000
    duration = distance/speed
    return duration


def poseCallback(data):
    global current_pose
    global duration

    target_pose = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
    print (target_pose)
    duration = computeDuration(current_pose, target_pose, speed)
    print("duration", duration)
    # Random 2-point trajectory
    
    x, y, z, ax, ay, az = target_pose
    v = PyKDL.Vector(x, y, z)
    
    # create a rotation from XYZ Euler angles
    r2 = PyKDL.Rotation.EulerZYX(math.radians(ax), math.radians(ay), math.radians(az))
    
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

    p.time_from_start = rospy.Duration(duration)
    
    goal = FollowCartesianTrajectoryGoal()
    goal.trajectory.points.append(p)

    client.send_goal(goal)
    client.wait_for_result()
    time = rospy.get_time()
    pub.publish(String("completed at: " + str(time)))



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

        target_pose = []
        current_pose = []
        speed = 100
        duration = 10

        pub = rospy.Publisher('/action_completed', String, queue_size=100)
        tf_listener = tf.TransformListener()
        listener(tf_listener)

    except rospy.ROSInterruptException:
         pass