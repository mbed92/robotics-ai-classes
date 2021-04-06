import numpy as np
import pybotics
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def create_joint_state_msg(joints: list):
    msg = JointState(header=Header(stamp=rospy.Time.now()))
    msg.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    msg.position = joints.copy()
    return msg


### [TODO IK]: Write a function that creates a kinematic chain in pybotics:
### * DH transform according to [rotX, transX, rotZ, TransZ]
### * For the reference: https://github.com/nnadeau/pybotics/blob/90e3503417300fc63b2f224a416c1f95369dc9b3/pybotics/predefined_models.py
def create_kinematic_model(dh: dict):
    ur_robot = None
    ###
    ...
    ###
    return ur_robot
