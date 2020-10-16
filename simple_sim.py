#!/usr/bin/env python

'''
Created on: 1 Sept 2020
Author: Olivier Kermorgant

Bridge to control arm position and velocity, simulates Baxter' kinematics

Subscribes:
- robot position or velocity command on topic /main_control/command

Publishes:
- robot joint positions commands on topic /joint_control/command (position control)
'''

# modules
# ROS stuff and multithreading
import roslib
import message_filters
import rospy,time,threading
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand
from baxter_core_msgs.srv import SolvePositionIK
# math
import numpy as np
from numpy.linalg import norm, inv, pinv
from urdf_parser_py.urdf import URDF
from tf_conversions import transformations

# system
import sys,os


verbose = True

# low-level sampling time
T = 1./50

# joint data: read URDF to get joint names and limits (position + velocity)
model = URDF.from_xml_string(rospy.get_param("/robot_description"))

class Joint:
    def __init__(self, joint):
        self.name = joint.name
        self.upper = joint.limit.upper
        self.lower = joint.limit.lower
        self.velocity = joint.limit.velocity*T
        
joints = dict([(joint.name, Joint(joint)) for joint in model.joints if joint.type in ('revolute','prismatic') and joint.mimic is None])

def sat(name, val):
    return min(max(val, joints[name].lower), joints[name].upper)

def sat_vel(name, val):
    return min(max(val, -joints[name].velocity), joints[name].velocity)

class JointGroup:
    def __init__(self, side = None):

        self.side = side        
        self.names = []
        
        if side is None:
            self.names = [name for name in joints if 'left' not in name and 'right' not in name]
        else:
            self.names = [name for name in joints if side in name]
        self.N = len(self.names)
        self.q = [0] * self.N
        self.v = [0] * self.N
        
        if side is None:
            return
        
        self.cmdCount = 0
        self.t0 = rospy.Time.now().to_sec()
        
        rospy.Subscriber('/robot/limb/{}/joint_command'.format(side), JointCommand, self.readBridgeCommand)

    def saturate(self, q, names = None):
        
        if names is None:    # standard ordering
            return [sat(self.names[i], q[i]) for i in range(self.N)]                
                
        saturated = []
        for i,name in enumerate(self.names):
            if name in names:
                idx = names.index(name)
                saturated.append(sat(self.names[i], q[idx]))
            else:
                saturated.append(self.q[i])
        return saturated 

    def followPosition(self, qDes, cmdCountCur):
        '''
        Follows the joint position setpoint as long as :
        - the corresponding position lies inside the joint limits
        - no other command has been received
        '''
        
        # go to qDes
        while (self.cmdCount == cmdCountCur) and not rospy.is_shutdown():
            
            # update position towards qDes
            for i in range(self.N):
                dq = qDes[i] - self.q[i]                                
                vmax = joints[self.names[i]].velocity
                if abs(dq) < vmax:
                    self.q[i] = qDes[i]
                    self.v[i] = 0
                else:
                   self.q[i] += np.sign(dq) * vmax
                   self.v[i] = np.sign(dq) * vmax/T
            time.sleep(T)        

    def followVelocity(self, data, cmdCountCur):
        '''
        Follows the joint velocity setpoint as long as :
        - the corresponding position lies inside the joint limits
        - no other command has been received
        '''
        qDot = []
        for i,name in enumerate(self.names):
            if name in data.names:
                idx = data.names.index(name)
                qDot.append(sat_vel(name, data.command[idx]*T))
            else:
                qDot.append(0)
        k = 0
                    
        while (self.cmdCount == cmdCountCur) and not rospy.is_shutdown():
            k = k+1
            # running setpoint
            new_q = self.saturate([self.q[i] + qDot[i] for i in range(self.N)])
            self.v = [(new_q[i]-self.q[i])/T for i in range(self.N)]
            self.q = new_q
            time.sleep(T)
  
    def readBridgeCommand(self, data):
        '''
        Execute command received on /robot/command topic
        '''
        if len(data.names) != len(data.command):
            return
      
        if self.cmdCount == 0:
            print('Switching to control mode')
        self.cmdCount += 1        
            
        self.t0 = rospy.Time.now().to_sec()  
            
        if data.mode == 2:
            # read velocities
            thread=threading.Thread(group=None,target=self.followVelocity, name=None, args=(data, self.cmdCount), kwargs={})
            thread.start()
        elif data.mode == 1:
                # read positions
                thread=threading.Thread(group=None,target=self.followPosition, name=None, args=(self.saturate(data.command, data.names), self.cmdCount), kwargs={})
                thread.start()

if __name__ == '__main__':

    # name of the node
    rospy.init_node('joint_control')
    
    elements = [JointGroup('left'), JointGroup('right'), JointGroup()]

    print('Waiting commands')
    
    state_pub = rospy.Publisher('/robot/joint_states', JointState, queue_size = 1)
    
    joint_states = JointState()
    joint_states.name = sum([elem.names for elem in elements], [])

    while not rospy.is_shutdown():
        
        joint_states.position = sum([elem.q for elem in elements], [])
        joint_states.velocity = sum([elem.v for elem in elements], [])
        joint_states.header.stamp = rospy.Time.now()
        state_pub.publish(joint_states)

        rospy.sleep(0.05)
