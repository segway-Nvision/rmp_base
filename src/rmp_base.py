#!/usr/bin/env python
import rospy
from system_defines import *
from user_event_handlers import RMPEventHandlers
import sys, time, threading, Queue, signal
from rmp_interface import RMPThread #Imports module. Not limited to modules in this pkg.
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
        # Setup subscriber
        rospy.init_node("rmp_base", anonymous = False)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.OnShutdown) # Shutdown behavior
        rospy.loginfo("Initialzing subscriber node [%s]." % (self.node_name))

        self.lock = threading.Lock() # Lock for the callback function of subVelCmd
        self.subVelCmd = rospy.Subscriber("/rmp220/base/vel_cmd", TwistStamped, self.SetVelocity, callback_args = (self.lock, vel))
        rospy.on_shutdown(self.OnShutdown) # Shutdown behavior
        rospy.loginfo("[%s] Initialzed." % (self.node_name))

    def SetVelocity(self, msg, args):
        lock = args[0]
        vel  = args[1]
        new_linear_vel  = msg.twist.linear.x
        new_angular_vel = msg.twist.angular.z

        # Critical section
        lock.acquire()
        if (abs(new_linear_vel) < MAX_LINEAR_VEL): # Update if the new velocity is in bound
            vel.twist.linear.x = new_linear_vel
        if (abs(new_angular_vel) < MAX_ANGULAR_VEL):
            vel.twist.angular.z = new_angular_vel
        rospy.loginfo("Velocity Components: [%f, %f]" % (vel.twist.linear.x, new_angular_vel))
        lock.release()

    def OnShutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))

def Shutdown(rmp_thread):
    print "Killing process..."
    in_flags.put(RMP_KILL) # Kill rmp_thread
    while (rmp_thread.isAlive()):
       pass

if __name__ == '__main__':
    # Setup communication thread
    rsp_queue = multiprocessing.Queue()
    cmd_queue = multiprocessing.Queue()
    in_flags  = multiprocessing.Queue()
    out_flags = multiprocessing.Queue()
    rmp_thread = RMPThread(rmp_addr, rsp_queue, cmd_queue, in_flags, out_flags, UPDATE_DELAY_SEC, LOG_DATA)
    #rmp_thread.setDaemon(True)
    rmp_thread.start()

    # Create event handler and set to BALANCE MODE
    vel = TwistStamped()
    event_handler = RMPEventHandlers(cmd_queue, rsp_queue, in_flags, vel)
    event_handler.GotoBalance()

    # Create the subscriber node
    node = rmp_base(vel)

    while not rospy.is_shutdown() and rmp_thread.isAlive():
        if not out_flags.empty():
            event_handler.handle_event[out_flags.get()]()
    Shutdown(rmp_thread)

    #Noriginal thread number after node init 4, after setup communication thread 5, after setup rmp_node sub,pub 7
