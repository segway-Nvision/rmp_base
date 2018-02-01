import rospy
import threading
from user_event_handlers import RMPEventHandlers
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

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
