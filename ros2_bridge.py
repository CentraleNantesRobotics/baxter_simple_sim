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
        if out:
            baxter_module = out[0]
            break
        
    baxter_lib = baxter_module.replace('/{}/site-packages'.format(python_version), '')
    
    os.environ['LD_LIBRARY_PATH'] = '{baxter}:/opt/ros/{distro}/opt/yaml_cpp_vendor/lib:/opt/ros/{distro}/opt/rviz_ogre_vendor/lib:/opt/ros/{distro}/lib/x86_64-linux-gnu:/opt/ros/{distro}/lib:'.format(baxter = baxter_lib, distro = ros2_distro) + os.environ['LD_LIBRARY_PATH']
    
    os.execv(sys.argv[0], sys.argv + ['__baxter_module:=' + baxter_module])
    print('End first run')
    sys.exit(0)
    
print('Second run')
print(os.environ['LD_LIBRARY_PATH'])


# import ROS 1 packages
import rospy
from baxter_core_msgs.msg import JointCommand as JointCommand1
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
from sensor_msgs.msg import JointState as JointState2

rospy.init_node('baxter_bridge')
rclpy.init(args=None)
node = rclpy.create_node('baxter_bridge')

# publishers
state_pub = node.create_publisher(JointState2, 'joint_states', 10)

command_pub = {}
for side in ('left', 'right'):
    command_pub[side] = rospy.Publisher('/robot/limb/{}/joint_command'.format(side), JointCommand1, queue_size=1)

# callback functions
def state_callback(msg1):
    global state_pub 
    
    msg2 = JointState2()
    msg2.name = msg1.name
    msg2.position = msg1.position
    msg2.velocity = msg1.velocity
    msg2.effort = msg1.effort
    msg2.header.stamp = node.get_clock().now().to_msg()
    state_pub.publish(msg2)
    
def command_callback(msg2, side):
    global command_pub 
    msg1 = JointCommand1()
    msg1.mode = msg2.mode
    msg1.command = msg2.command
    msg1.names = msg2.names
    
    command_pub[side].publish(msg1)
    
    
# subscribers
command_sub = {}
for side in ('left', 'right'):
    command_sub[side] = node.create_subscription(JointCommand2, '/robot/limb/{}/joint_command'.format(side), lambda msg: command_callback(msg, side), 10)
    
state_sub = rospy.Subscriber('joint_states', JointState1, state_callback)

rate = rospy.Rate(50)


while rclpy.ok(): #not rospy.is_shutdown():
    rclpy.spin_once(node)
    rate.sleep()

print('Exit loop')

#rospy.signal_shutdown('ROS 2 shutdown')
rclpy.shutdown()
