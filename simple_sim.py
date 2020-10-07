#!/usr/bin/env python

'''
Created on: 12 Sep 2012
Author: Olivier Kermorgant

Bridge to control arm position and velocity 

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
# math
from pylab import arange, sign
# system
import sys,os


verbose = True

# low-level sampling time
T = 1./50

# joint data: read URDF to get joint names and limits (position + velocity)
urdf = rospy.get_param("/robot_description").splitlines()
N = 0

joint_specs = {}
inJoint = False
jName = ''
for ui in urdf:
    if inJoint:
        if 'limit' in ui:
            joint_specs[jName] = {}
            s = ui.split('"')
            for i in range(len(s)):
                if 'lower' in s[i]:
                    joint_specs[jName]['lower'] = float(s[i+1])
                elif 'upper' in s[i]:
                    joint_specs[jName]['upper'] = float(s[i+1])
                elif 'velocity' in s[i]:
                    joint_specs[jName]['velocity'] = T*float(s[i+1])
            inJoint = False
        elif 'mimic' in ui:
            inJoint = False
    else:
        if 'joint name=' in ui and 'fixed' not in ui:
                jName = ui.split('"')[1]
                inJoint = True
                
def sat(name, val):
    return min(max(val, joint_specs[name]['lower']), joint_specs[name]['upper'])

def sat_vel(name, val):
    return min(max(val, -joint_specs[name]['velocity']), joint_specs[name]['velocity'])

class Joints:
    def __init__(self, side = None):

        self.side = side
        
        self.names = []
        
        if side is None:
            self.names = [name for name in joint_specs if 'left' not in name and 'right' not in name]
        else:
            self.names = [name for name in joint_specs if side in name]
        self.N = len(self.names)
        self.q = [0] * self.N
        self.v = [0] * self.N
        
        if side is None:
            return
        
        self.cmdCount = 0
        self.t0 = rospy.Time.now().to_sec()
        
        rospy.Subscriber('/robot/limb/{}/joint_command'.format(side), JointCommand, self.readBridgeCommand)
                
    def spec(self, i, prop):
        return joint_specs[self.names[i]][prop]
    
    def saturate(self, q, names = None):
        
        if names is None:    # standard ordering
            return [sat(self.names[i], q[i]) for i in range(self.N)]                
                
        saturated = []
        for i,name in enumerate(self.names):
            if name in names:
                idx = names.index(name)
                lower, upper = self.spec(i, 'lower'), self.spec(i, 'upper')
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
                vmax = self.spec(i, 'velocity')
                if abs(dq) < vmax:
                    self.q[i] = qDes[i]
                    self.v[i] = 0
                else:
                   self.q[i] += sign(dq) * vmax
                   self.v[i] = sign(dq) * vmax/T
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
    '''
    Begin of main code
    '''
    
    # name of the node
    rospy.init_node('joint_control')
    
    elements = [Joints('left'), Joints('right'), Joints()]

    print('Waiting commands')
    
    state_pub = rospy.Publisher('/joint_states', JointState, queue_size = 1)
    
    joint_states = JointState()
    joint_states.name = sum([elem.names for elem in elements], [])

    while not rospy.is_shutdown():
        
        joint_states.position = sum([elem.q for elem in elements], [])
        joint_states.velocity = sum([elem.v for elem in elements], [])
        joint_states.header.stamp = rospy.Time.now()
        state_pub.publish(joint_states)

        rospy.sleep(0.05)
