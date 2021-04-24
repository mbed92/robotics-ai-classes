import numpy as np
import rospy
import tf
from pybotics.robot import MDHKinematicChain, Robot
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def create_joint_state_msg(joints: list):
    msg = JointState(header=Header(stamp=rospy.Time.now()))
    msg.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    msg.position = joints.copy()
    return msg


def get_transform(parent_tf, child_tf, tf_listaner):
    trans, rot = None, None
    try:
        tf_listaner.waitForTransform(parent_tf, child_tf, rospy.Time(), rospy.Duration(5.0))
        (trans, rot) = tf_listaner.lookupTransform(parent_tf, child_tf, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Cannot find transformation between {] and {}".format(parent_tf, child_tf))
    return trans, rot

### [TODO IK]: Write a function that creates a kinematic chain in pybotics:
### * DH transform according to MDH [rotX, transX, rotZ, TransZ]
### * For the reference: https://github.com/nnadeau/pybotics/blob/90e3503417300fc63b2f224a416c1f95369dc9b3/pybotics/predefined_models.py
### * For the reference: https://github.com/nnadeau/pybotics/blob/90e3503417300fc63b2f224a416c1f95369dc9b3/examples/kinematics.ipynb
def create_kinematic_model(dh: dict):
    ur_robot = None
    ###
    ...
    ###
    return ur_robot
