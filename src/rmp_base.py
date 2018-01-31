#!/usr/bin/env python
import sys, time, threading, signal
from system_defines import *
from user_event_handlers import RMPEventHandlers
from rmp_ros_wrapper import MotionCmdSubscriber

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

if __name__ == '__main__':
    # Create event handler, start a rmp thread and set rmp to BALANCE MODE
    event_handler = RMPEventHandlers(rmp_addr, UPDATE_DELAY_SEC, LOG_DATA)
    event_handler.start_thread()
    event_handler.goto_balance()
    out_flags = event_handler.out_flags
    rmp_thread = event_handler.rmp_thread

    # Create the subscriber node
    node = MotionCmdSubscriber(event_handler)

    while not rospy.is_shutdown() and rmp_thread.isAlive():
        if not out_flags.empty():
            event_handler.handle[out_flags.get()]()
    print "Killing process..."
    event_handler.kill_thread()

    #Noriginal thread number after node init 4, after setup communication thread 5, after setup rmp_node sub,pub 7
