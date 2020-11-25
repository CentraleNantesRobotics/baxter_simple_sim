#!/usr/bin/env python

import sys, os
import signal
import subprocess


depth = 0
if '-d' in sys.argv:
    depth = int(sys.argv[sys.argv.index('-d')+1])
    
def display(s):
    print(' '*depth + 'depth = {} - '.format(depth) + s)
    
#print('Running with depth {}'.format(depth))
    
if depth == 3:
    sys.exit(0)
    
def rerun(add_args, env = None):
        
    argv = sys.argv[:]
    
    if '-d' in argv:
        idx = argv.index('-d')
        argv.pop(idx)
        argv.pop(idx)
    
    argv += add_args.split() + ['-d',str(depth+1)]
        
    for key in ('--ros-args', '--remap'):
        if key in argv:
            argv.remove(key)

    has_name = False
    for i,arg in enumerate(argv):
        if arg.startswith('__name'):
            has_name = True
            break

    if has_name:
        argv.append('--ros-args')
        argv.append('--remap')
        argv.append(argv.pop(i))
                
    if env is None:
        pid = subprocess.Popen(argv)
    else:
        pid = subprocess.Popen(argv, env=env)
        
    def signal_handler(sig, frame):
        #display('received SIGINT')
        pid.send_signal(signal.SIGINT)
        #pid.kill()
        
    signal.signal(signal.SIGINT, signal_handler)

    try:
        pid.wait()
    except KeyboardInterrupt:
        pid.send_signal(signal.SIGINT)
        pid.wait() 
        
    sys.exit(0)

ros_distro = os.listdir('/opt/ros')
ros1_distro,ros2_distro = max(ros_distro), min(ros_distro)
python_version = 'python3.8'

# check if baxter_module is passed as argument
baxter_module = ''
for arg in sys.argv:
    if arg.startswith('__baxter_module'):
        baxter_module = arg.split(':=')[1]
        break
    
remove_path = '-p' in sys.argv
    
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
    
    hybrid_env = os.environ.copy()
    hybrid_env['LD_LIBRARY_PATH'] = '{baxter}:/opt/ros/{distro}/opt/yaml_cpp_vendor/lib:/opt/ros/{distro}/opt/rviz_ogre_vendor/lib:/opt/ros/{distro}/lib/x86_64-linux-gnu:/opt/ros/{distro}/lib:'.format(baxter = baxter_lib, distro = ros2_distro) + hybrid_env['LD_LIBRARY_PATH']
    
    rerun('__baxter_module:=' + baxter_module, env = hybrid_env)

# import ROS 1 packages
import rospy
from baxter_core_msgs.msg import JointCommand as JointCommand1
from baxter_core_msgs.srv import SolvePositionIK as SolvePositionIK1
from baxter_core_msgs.srv import SolvePositionIKRequest as SolvePositionIKRequest1
from sensor_msgs.msg import JointState as JointState1
from geometry_msgs.msg import PoseStamped as PoseStamped1

# delete ROS 1 paths
ros1_msgs = set()
for key in list(sys.modules.keys()):
    if '.msg' in key or '.srv' in key:
        ros1_msgs.add(key)
        ros1_msgs.add(key[:key.find('.')])
for key in ros1_msgs:
    sys.modules.pop(key)
if remove_path:
    for key in list(sys.path):
        for pref in (ros1_distro, 'ros', 'ros1'):
            if pref + '/' in key and key in sys.path:
                sys.path.remove(key)

# add ROS 2 modules
sys.path.insert(2, '/opt/ros/{}/lib/{}/site-packages'.format(ros2_distro, python_version))
sys.path.insert(2, baxter_module)

import rclpy
from baxter_core_msgs.msg import JointCommand as JointCommand2
from baxter_core_msgs.srv import SolvePositionIK as SolvePositionIK2
from sensor_msgs.msg import JointState as JointState2

rclpy.init(args=None)
node = rclpy.create_node('baxter_bridge')

class ArmRelay:
    def __init__(self, side):
        self.side = side
        self.command_pub = rospy.Publisher('/robot/limb/{}/joint_command'.format(side), JointCommand1, queue_size=1)        
        self.command_sub = node.create_subscription(JointCommand2, '/robot/limb/{}/joint_command'.format(side), self.command_callback, 10)
        self.msg1 = JointCommand1()
        service_name = '/ExternalTools/{}/PositionKinematicsNode/IKService'.format(side)
        
        rospy.wait_for_service(service_name)
        self.ik_client1 = rospy.ServiceProxy(service_name, SolvePositionIK1)
        self.ik_req1 = SolvePositionIKRequest1()
        self.ik_req1.seed_mode = 0
        self.ik_req1.pose_stamp = [PoseStamped1()]
        
        self.ik_server2 = node.create_service(SolvePositionIK2, service_name, self.ik_callback)
        
    def command_callback(self, msg2):
        self.msg1.mode = msg2.mode
        self.msg1.command = msg2.command
        self.msg1.names = msg2.names
        self.command_pub.publish(self.msg1)
        
    def ik_callback(self, req, res):
        print('Got request {}'.format(req))
        
        if len(req.pose_stamp) == 0:
            return res
        
        # write ROS 1 request
        print('To ROS 1')
        #self.ik_req1.pose_stamp[0].position.x = 
        for field,axes in (('position','xyz'), ('orientation','xyzw')):
            for ax in axes:
                value = getattr(getattr(req.pose_stamp[0].pose, field), ax)
                setattr(getattr(self.ik_req1.pose_stamp[0].pose, field), ax, value)
        # call ROS 1 service
        print('calling')
        res1 = self.ik_client1(self.ik_req1)
        print('to ROS 2')
        # write ROS 2 response
        solution = JointState2()
        solution.name = res1.joints[0].name
        solution.position = res1.joints[0].position
        res.joints = [solution]
        res.is_valid = [res1.isValid[0]]
        print('return {}'.format(res))
        return res

# /joint_states relay 1 -> 2

stop = False
try:
    
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
        
    rospy.init_node('baxter_bridge')
    state_sub = rospy.Subscriber('/robot/joint_states', JointState1, state_callback)

    # command relay 2 -> 1
    left_relay = ArmRelay('left')
    right_relay = ArmRelay('right')

    rate = rospy.Rate(50)


    def signal_handler(sig, frame):
        global stop
        if stop:
            return

        #display('got sigint when stop = {}'.format(stop))
        stop = True
                
    signal.signal(signal.SIGINT, signal_handler)

    while not stop:
        #display('In loop - spin')
        rclpy.spin_once(node, timeout_sec = 0)
        #display('In loop - sleep')
        rate.sleep()
        
    #display('after loop -- killing ROS 2')
    node.destroy_node()
    rclpy.shutdown()
    #display('killing ROS 1')
    rospy.signal_shutdown('')
    #display('exit')
    sys.exit(0)

except:
    ##display('Catch exception')
    if not stop:
        node.destroy_node()
        rerun('__baxter_module:=' + baxter_module + ' -p')
