#!/usr/bin/env python

import sys, os

ros2_distro = 'foxy'
python_version = 'python3.8'

# check if baxter_module is passed as argument
baxter_module = ''
for arg in sys.argv:
    if arg.startswith('__baxter_module'):
        baxter_module = arg.split(':=')[1]
        break
    
if baxter_module == '':
    # first run -> update env and re-run
    
    # locate ROS 2 baxter_core_msgs install
    import subprocess
    for path in ('~', '/opt'):
        proc = subprocess.Popen(['/bin/bash', '-E'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)        
        out = proc.communicate(bytes('find {} -wholename *install/baxter_core_msgs/lib/{}/site-packages 2>/dev/null'.format(path, python_version), 'utf-8'))[0].decode('utf-8').splitlines()
        for path in out:
            if '/.' not in path:
                baxter_module = path
                break
        if baxter_module:
            break
        
    baxter_lib = baxter_module.replace('/{}/site-packages'.format(python_version), '')
    
    os.environ['LD_LIBRARY_PATH'] = '{baxter}:/opt/ros/{distro}/opt/yaml_cpp_vendor/lib:/opt/ros/{distro}/opt/rviz_ogre_vendor/lib:/opt/ros/{distro}/lib/x86_64-linux-gnu:/opt/ros/{distro}/lib:'.format(baxter = baxter_lib, distro = ros2_distro) + os.environ['LD_LIBRARY_PATH']
    
    os.execv(sys.argv[0], sys.argv + ['__baxter_module:=' + baxter_module])
    sys.exit(0)

# import ROS 1 packages
import rospy
from baxter_core_msgs.msg import JointCommand as JointCommand1
from baxter_core_msgs.srv import SolvePositionIK as SolvePositionIK1
from sensor_msgs.msg import JointState as JointState1

# override modules
for key in list(sys.modules.keys()):
    if '_msg' in key:
        sys.modules[key.replace('_msg', '_msg1')] = sys.modules.pop(key)

# add ROS 2 modules
sys.path.insert(2, '/opt/ros/{}/lib/{}/site-packages'.format(ros2_distro, python_version))
sys.path.insert(2, baxter_module)

import rclpy
from baxter_core_msgs.msg import JointCommand as JointCommand2
from baxter_core_msgs.srv import SolvePositionIK as SolvePositionIK2
from sensor_msgs.msg import JointState as JointState2

rospy.init_node('baxter_bridge')
rclpy.init(args=None)
node = rclpy.create_node('baxter_bridge')

class ArmRelay:
    def __init__(self, side):
        self.side = side
        self.command_pub = rospy.Publisher('/robot/limb/{}/joint_command'.format(side), JointCommand1, queue_size=1)        
        self.command_sub = node.create_subscription(JointCommand2, '/robot/limb/{}/joint_command'.format(side), self.command_callback, 10)
        self.msg1 = JointCommand1()
        
    def command_callback(self, msg2):
        self.msg1.mode = msg2.mode
        self.msg1.command = msg2.command
        self.msg1.names = msg2.names
        self.command_pub.publish(self.msg1)      

# /joint_states relay 1 -> 2
state_pub = node.create_publisher(JointState2, '/robot/joint_states', 10)

# callback functions
def state_callback(msg1):
   
    msg2 = JointState2()
    msg2.name = msg1.name
    msg2.position = msg1.position
    msg2.velocity = msg1.velocity
    msg2.effort = msg1.effort
    msg2.header.stamp = node.get_clock().now().to_msg()
    state_pub.publish(msg2)
    
state_sub = rospy.Subscriber('/robot/joint_states', JointState1, state_callback)

# command relay 2 -> 1
left_relay = ArmRelay('left')
right_relay = ArmRelay('right')

rate = rospy.Rate(50)

while not rospy.is_shutdown():
    rclpy.spin_once(node)
    rate.sleep()

rclpy.shutdown()
