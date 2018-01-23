#!/usr/bin/env python
import rospy
from system_defines import *
from user_event_handlers import RMPEventHandlers
import sys, time, threading, Queue, signal
from rmp_interface import RMP #Imports module. Not limited to modules in this pkg.
from std_msgs.msg import String #Imports msg

#for rmp220
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, JointState
from rmp_msgs.msg import BoolStamped, AudioCommand, FaultStatus
import multiprocessing


"""
Define the update delay or update period in seconds. Must be greater
than the minimum of 0.01s
"""
UPDATE_DELAY_SEC = 0.05

"""
Define whether to log the output data in a file. This will create a unique CSV
file in ./RMP_DATA_LOGS with the filename containing a time/date stamp
"""
LOG_DATA = True

"""
The platform address may be different than the one in your config
(rmp_config_params.py). This would be the case if you wanted to update
ethernet configuration. If the ethernet configuration is updated the
system needs to be power cycled for it to take effect and this should
be changed to match the new values you defined in your config
"""
rmp_addr = ("192.168.0.40", 8080) # This is the default value and matches the config

"""
The limit of the maximum velocity
"""
MAX_LINEAR_VEL = 0.6
MAX_ANGULAR_VEL = 3.15

class rmp_base(object):
    def __init__(self, vel):
        # Name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("Initialzing subscriber node [%s]." % (self.node_name))

        # Setup subscriber
        rospy.init_node('rmp_base', anonymous = False)
        self.subVelCmd = rospy.Subscriber("/rmp220/base/vel_cmd", TwistStamped, self.Callback, callback_args = (self.lock, vel))
        self.lock = threading.Lock() # Lock for the callback function of subVelCmd
        rospy.on_shutdown(self.OnShutdown) # Shutdown behavior
        rospy.loginfo("[%s] Initialzed." % (self.node_name))

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value) # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def Callback(self, msg, args):
        lock = args[0]
        vel  = args[1]

        thread = threading.Thread(target = self.SetVelocity, args = (msg, lock, vel))
        thread.setDaemon(True)
        thread.start()

    def SetVelocity(self, msg, lock, vel):
        new_linear_vel  = msg.twist.linear.x
        new_angular_vel = msg.twist.angular.z

        lock.acquire()
        # Critical section
        # Update if the new velocity is in bound
        if (abs(new_linear_vel) < MAX_LINEAR_VEL):
            vel.twist.linear.x = new_linear_vel
        if (abs(new_angular_vel) < MAX_ANGULAR_VEL):
            vel.twist.angular.z = new_angular_vel
        rospy.loginfo("Velocity Components: [%f, %f]" % (vel.twist.linear.x, new_angular_vel))
        lock.release()

    def OnShutdown(self, in_flags):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))

def Shutdown(in_flags):
    in_flags.put(RMP_KILL)
    sys.exit(0)

if __name__ == '__main__':
    # Setup communication thread
    rsp_queue = multiprocessing.Queue()
    cmd_queue = multiprocessing.Queue()
    in_flags  = multiprocessing.Queue()
    out_flags = multiprocessing.Queue()
    rmp_thread = threading.Thread(target = RMP, args = (rmp_addr, rsp_queue, cmd_queue, in_flags, out_flags, UPDATE_DELAY_SEC, LOG_DATA))
    rmp_thread.setDaemon(True)
    rmp_thread.start()

    # Kill rmp_thread on shutting down
    signal.signal(signal.SIGINT, signal_handler)

    # Setup initial velocity
    vel = TwistStamped()
    vel.twist.linear.x = 0.0
    vel.twist.angular.z = 0.0

    # Create the subscriber node
    node = rmp_base(vel)

    # Create event handler and set to BALANCE MODE
    event_handler = RMPEventHandlers(cmd_queue, rsp_queue, in_flags)
    event_handler.GotoBalance()
    rospy.loginfo("Initialization finished!")

    while not rospy.is_shutdown():
        event_handler.Send_MotionCmd(vel.twist.linear.x, vel.twist.angular.z)
        time.sleep(UPDATE_DELAY_SEC)


    #Noriginal thread number after node init 4, after setup communication thread 5, after setup rmp_node sub,pub 7
