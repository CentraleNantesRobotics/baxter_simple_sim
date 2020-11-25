#!/usr/bin/env python

'''
Created on: 1 Sept 2020
Author: Olivier Kermorgant

Bridge to control arm position and velocity, simulates Baxter' kinematics

Subscribes:
- robot position or velocity command on topic /main_control/command

Publishes:
- robot joint positions commands on topic /joint_control/command (position control)

If lab param is in (puppet, mirror), right arm moves autonomously
'''

# modules
# ROS stuff and multithreading
import roslib
import message_filters
import rospy,time,threading
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest, SolvePositionIKResponse
from geometry_msgs.msg import PoseStamped, TransformStamped
from baxter_core_msgs.msg import JointCommand
import tf2_ros

# math
import numpy as np
from numpy.linalg import norm, inv, pinv
from urdf_parser_py.urdf import URDF
from tf_conversions import transformations

# system
import sys,os


    

if __name__ == '__main__':

    # name of the node
    rospy.init_node('ik_client')
        
    
    rospy.wait_for_service('/ExternalTools/right/PositionKinematicsNode/IKService')
    ikr = rospy.ServiceProxy('/ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
    ikl = rospy.ServiceProxy('/ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
    
    pubr = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size = 1)
    publ = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size = 1)
    
    cmd = JointCommand()
    cmd.mode = 1
    
    req = SolvePositionIKRequest()
    req.seed_mode = 0
    
    M0 = transformations.quaternion_matrix((0.031, 0.808,.587,0.028))
    M0[0,3] = 0.7
    M0[1,3] = -0.137
    M0[2,3] = 0.357
    
    Md = np.eye(4)
    
    Mrl = np.zeros((4,4))
    Mrl[2,3] = 0.2
    Mrl[2,0] = Mrl[0,2] = -1
    Mrl[1,1] = 1
    Mrl[3,3] = 1
    
    print(Mrl,matrix_msg(Mrl))
    
    t = 0
    dt = 0.1
    
    br = tf2_ros.TransformBroadcaster()
    tr = TransformStamped()
    tr.header.stamp = rospy.Time.now()
    tr.header.frame_id = "right_gripper"
    tr.child_frame_id = 'left_gripper_target'
    tr.transform.translation.z = .2
    tr.transform.rotation.y = 1

    
    while not rospy.is_shutdown():
        tr.header.stamp = rospy.Time.now()                    
        br.sendTransform(tr)
        # update pose
        
        Md[0,3] = 0.05*np.cos(t/2.)
        Md[1,3] = 0.1*np.sin(t/2)
        Md[2,3] = 0.2*np.cos(t/2)
        Md[:3,:3] = transformations.euler_matrix(.1*np.cos(t), .1*np.sin(t/2), .05*np.cos(t+4))[:3,:3]
         
        M = np.dot(M0,Md)    
        
        req.pose_stamp = [matrix_msg(M)]
        
        res = ikr(req)        
        cmd.names = res.joints[0].name
        cmd.command = res.joints[0].position
        pubr.publish(cmd)
        
        req.pose_stamp = [matrix_msg(np.dot(M, Mrl))]
        res = ikl(req)        
        cmd.names = res.joints[0].name
        cmd.command = res.joints[0].position
        publ.publish(cmd)
        
        t += dt
        rospy.sleep(dt)
        
