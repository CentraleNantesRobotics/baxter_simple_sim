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
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest, SolvePositionIKResponse
from geometry_msgs.msg import PoseStamped

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
    def __init__(self, side = None, lab = None):

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
        
        if lab is None or side == 'left':
            rospy.Subscriber('/robot/limb/{}/joint_command'.format(side), JointCommand, self.readBridgeCommand)
        elif lab == 'mirror' and side == 'right':
            thread=threading.Thread(group=None,target=self.mirrorMove, name=None, kwargs={})
            thread.start()
        
        if lab == 'puppet':
            if side == 'left':
                self.q = [-0.10266471341792811, -0.05772519794064627, -0.36909985265803286, 0.7955637684523007, -1.1302109919350245, 2.0871143114989947, 2.559173568111138]
                return
            
            # init right arm IK service
            self.q = [0.05183482296524736, -0.8682037556901855, 0.9476424476835772, 1.685625905154571, 0.7588301888902473, 0.999285765226626, -0.3354836773966444]
                
            thread=threading.Thread(group=None,target=self.puppetMove, name=None, kwargs={})
            thread.start()

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
    
    def mirrorMove(self):
        low = [joints[name].lower for name in self.names]
        up = [joints[name].upper for name in self.names]
        mid = [.5*(low[i]+up[i]) for i in range(self.N)]
        rng = [up[i]-mid[i] for i in range(self.N)]
        
        t = 0
        while not rospy.is_shutdown():
            for i in range(self.N):
                self.q[i] = mid[i] + 0.1*(i+1)*rng[i]*np.cos(0.1*(i+2)*t)
            time.sleep(T)
            t += T
        
    def puppetMove(self):
        print('init puppet move')
        
        rospy.wait_for_service('/ExternalTools/right/PositionKinematicsNode/IKService')
        ik = rospy.ServiceProxy('/ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
        
        req = SolvePositionIKRequest()
        req.seed_mode = 0
        req.pose_stamp = [PoseStamped()]
    
        M0 = transformations.quaternion_matrix((0.031, 0.808,.587,0.028))
        M0[0,3] = 0.7
        M0[1,3] = -0.137
        M0[2,3] = 0.357
        
        t = 0
        while not rospy.is_shutdown():
            # variation around work pose
            Md = transformations.euler_matrix(.1*np.cos(t), .1*np.sin(t/2), .05*np.cos(t+4))
            Md[0,3] = 0.05*np.cos(t/2.)
            Md[1,3] = 0.1*np.sin(t/2)
            Md[2,3] = 0.2*np.cos(t/2)
            M = np.dot(M0, Md)

            req.pose_stamp[0].pose.position.x = M[0,3]
            req.pose_stamp[0].pose.position.y = M[1,3]
            req.pose_stamp[0].pose.position.z = M[2,3]
            
            q = transformations.quaternion_from_matrix(M)
            q = q/np.linalg.norm(q)
            req.pose_stamp[0].pose.orientation.x = q[0]
            req.pose_stamp[0].pose.orientation.y = q[1]
            req.pose_stamp[0].pose.orientation.z = q[2]
            req.pose_stamp[0].pose.orientation.w = q[3]   
            req.pose_stamp[0].header.stamp = rospy.Time.now()
            
            self.q = list(ik(req).joints[0].position)
            time.sleep(T)
            t += T

    def followPosition(self, qDes, cmdCountCur):
        '''
        Follows the joint position setpoint as long as :
        - the corresponding position lies inside the joint limits
        - no other command has been received
        '''
        
        print('Got position request: {}'.format(qDes))
        
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
    
    lab = None
    if rospy.has_param('~lab'):
        lab = rospy.get_param('~lab')
        if lab not in ('puppet', 'mirror'):
            lab = None
    
    elements = [JointGroup('left', lab), JointGroup('right', lab), JointGroup()]

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
