#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import TwistStamped

def talker():
    pub = rospy.Publisher("/rmp220/base/vel_cmd", TwistStamped, queue_size=100)
    rospy.init_node("segway_controller", anonymous=True)

    count = 0
    while count < 5:
        vel = TwistStamped()
        vel.twist.linear.x  = 0.0
        vel.twist.angular.z = 0.0
        pub.publish(vel)
        count += 1
        print("count = %d" % count)
        time.sleep(2)

    """
    vel.twist.linear.x  = 0.4
    vel.twist.angular.z = 0.785
    pub.publish(vel)
    time.sleep(2) # 0.333 for 15 deg

    vel.twist.linear.x  = 0.2
    vel.twist.angular.z = 0.0
    pub.publish(vel)
    time.sleep(10) # 0.333 for 15 deg
    """

    """
    vel = TwistStamped()
    vel.twist.linear.x  = 0.2
    vel.twist.angular.z = 0.0
    pub.publish(vel)
    time.sleep(10)
    """

    vel.twist.linear.x  = 0.0
    vel.twist.angular.z = 0.0
    pub.publish(vel)

    rospy.loginfo("End of publishing.")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
