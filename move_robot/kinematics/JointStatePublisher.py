import rospy
from sensor_msgs.msg import JointState
import message_filters

from kinematics.utilities import create_joint_state_msg


class JointStatePublisher:
    def __init__(self):
        self.rate = rospy.get_param("~rate", 10)
        inv_kin_topic = rospy.get_param("~inv_kin_joint_states", "inv_kin_joint_states")
        joint_state_topic = rospy.get_param("~joint_states_topic", "joint_states")

        ### [TODO IK]: Write a publisher that publishes JointState message type on a joint_state_topic topic
        self.j_pub = ...

        ### [TODO IK]: Write a subscriber:
        ### * do not register callback
        ### * use message filters http://wiki.ros.org/message_filters to create cache
        self.j_sub = ...
        self.j_cache = ...

        ### [TODO IK]: Create some initial state message that contains joint angles:
        ### * you might use create_joint_state_msg(...) function for that
        self.state = ...

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            ### [TODO IK]: implement loop that publishes self.state when it changes:
            ### * get last cached message from created self.j_cache Cache object
            ### * remember to update self.state.header.stamp with a current time
            ...
            ###
            rate.sleep()
