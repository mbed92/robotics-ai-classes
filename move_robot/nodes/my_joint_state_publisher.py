#!/usr/bin/env python3

import rospy

from kinematics import JointStatePublisher

if __name__ == "__main__":
    rospy.init_node("my_joint_state_publisher")
    state_publisher = JointStatePublisher()
    state_publisher.run()
