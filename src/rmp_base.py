#!/usr/bin/env python
import rospy
from system_defines import *
from user_event_handlers import RMPEventHandlers
import sys, time, threading, signal
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

class MotionCmdSubscriber(object):
    def __init__(self, event_handler):
        # Setup subscriber
        rospy.init_node("rmp_base", anonymous = False)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown_node) # Shutdown behavior
        rospy.loginfo("Initialzing subscriber node [%s]." % (self.node_name))

        self.lock          = threading.Lock() # Lock for the callback function of subVelCmd
        self.event_handler = event_handler
        self.sub_vel_cmd   = rospy.Subscriber("/rmp220/base/vel_cmd", TwistStamped, self.set_velocity, callback_args = (self.lock, self.event_handler))
        rospy.loginfo("[%s] Initialzed." % (self.node_name))

    def set_velocity(self, msg, args):
        lock            = args[0]
        event_handler   = args[1]

        # Critical section
        lock.acquire()
        event_handler.update_vel(msg)
        rospy.loginfo("Velocity Components: [%f, %f]" % (event_handler.vel.twist.linear.x, event_handler.vel.twist.angular.z))
        lock.release()

    def shutdown_node(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))

if __name__ == '__main__':
    # Setup communication thread
    rsp_queue = multiprocessing.Queue()
    cmd_queue = multiprocessing.Queue()
    in_flags  = multiprocessing.Queue()
    out_flags = multiprocessing.Queue()
    rmp_thread = RMPThread(rmp_addr, rsp_queue, cmd_queue, in_flags, out_flags, UPDATE_DELAY_SEC, LOG_DATA)
    rmp_thread.start()

    # Create event handler and set to BALANCE MODE
    event_handler = RMPEventHandlers(cmd_queue, rsp_queue, in_flags, rmp_thread)
    event_handler.handle_event[RMP_GOTO_BALANCE]()

    # Create the subscriber node
    node = MotionCmdSubscriber(event_handler)

    while not rospy.is_shutdown() and rmp_thread.isAlive():
        if not out_flags.empty():
            event_handler.handle_event[out_flags.get()]()
    print "Killing process..."
    event_handler.handle_event[RMP_KILL]()

    #Noriginal thread number after node init 4, after setup communication thread 5, after setup rmp_node sub,pub 7
