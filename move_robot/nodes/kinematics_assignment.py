#!/usr/bin/env python3

import rospy

from kinematics import KinematicsManager

if __name__ == "__main__":
    rospy.init_node("kinematics_assignment")
    kin = KinematicsManager()
    kin.run()
