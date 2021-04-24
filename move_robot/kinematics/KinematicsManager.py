import rospy
import tf
import yaml
from geometry_msgs.msg import Point, Quaternion, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from .utilities import create_joint_state_msg, create_kinematic_model, get_transform


class KinematicsManager:

    # [TODO IK]: inverse kinematics callback - executes when you move the interactive marker in rviz
    def calculate_inv_kin(self, msg):
        if self.mode == Modes.INVERSE_KIN:
            ### write your own inverse kinematics equations
            thetas = ...
            ###

            if thetas is not None and None not in thetas:
                j_msg = create_joint_state_msg(thetas)
                self.inv_kin_joints_pub.publish(j_msg)
            else:
                rospy.logwarn("Cannot write joint thetas {}".format(thetas))

    # [TODO FK]: forward kinematics callback - executes when you move the joint from gui
    def calculate_fwd_kin(self, msg):
        if self.mode == Modes.FODWARD_KIN:
            ### write your own forward kinematics equations
            coordinates = ...
            ###

            if None not in coordinates:
                self.create_non_interactive_marker(coordinates)
            else:
                rospy.logwarn("Cannot write marker coordinates {}".format(coordinates))

    def __init__(self):
        self.mode = rospy.get_param("~mode", "")
        self.attach_interactive_marker_tf = rospy.get_param("~attach_interactive_marker_tf", "")
        self.base_tf = rospy.get_param("~base_tf", "")
        self.robot_joints = rospy.get_param("~joint_states", "")
        self.joint_state_topic = rospy.get_param("~inv_kin_joint_states", "")
        self.pose_state_topic = rospy.get_param("~fwd_kin_marker_pose", "")
        self.ur_config = rospy.get_param("~ur_config", "")
        self.listener = tf.TransformListener()

        ### [TODO FK]: load dh parameters according to: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
        ### * look into the config/ur3_dh.yaml file
        stream = open(self.ur_config, 'r')
        self.dh = yaml.load(stream, Loader=yaml.FullLoader)

        ### [TODO IK]: implement function that takes DH parameters and creates the kinematic model in pybotics
        self.model = create_kinematic_model(self.dh)

        # setup markers
        self.i_marker = InteractiveMarker()
        self.dot_marker = Marker()
        self.dot_control = InteractiveMarkerControl()
        self.m_marker_server = InteractiveMarkerServer("input_pose")
        self.fwd_kin_marker = Marker()

        # setup assignment-related environemnt
        self.inv_kin_joints_pub = None
        self.fwd_kin_marker_pub = None
        self.fwd_kin_marker_sub = None
        if self.mode == Modes.INVERSE_KIN:
            self.create_interactive_marker()

            ### [TODO IK]: write a publisher for joint states that takes self.joint_state_topic topic and publishers JointState messages
            self.inv_kin_joints_pub = ...

        elif self.mode == Modes.FODWARD_KIN:

            ### [TODO FK]: write:
            ### * a publisher that publishers a Marker message type on the self.pose_state_topic topic
            ### * a Subscriber that takes JointState message type from the self.robot_joints topic and sets the method self.calculate_fwd_kin as a callback
            self.fwd_kin_marker_pub = ...
            self.fwd_kin_marker_sub = ...
            self.create_non_interactive_marker([1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        else:
            rospy.logerr("Unknown mode param. Not initialized. ")

    def create_non_interactive_marker(self, coordinates: list):
        self.fwd_kin_marker.header.frame_id = self.base_tf
        self.fwd_kin_marker.header.stamp = rospy.Time.now()
        self.fwd_kin_marker.type = self.fwd_kin_marker.CUBE
        self.fwd_kin_marker.action = self.fwd_kin_marker.ADD
        self.fwd_kin_marker.scale = Vector3(0.05, 0.05, 0.05)
        self.fwd_kin_marker.color = ColorRGBA(0.3, 0.7, 0.3, 0.7)
        self.fwd_kin_marker.pose.position = Point(*coordinates[:3])
        self.fwd_kin_marker.pose.orientation = Quaternion(*coordinates[3:])
        self.fwd_kin_marker_pub.publish(self.fwd_kin_marker)

    def create_interactive_marker(self):
        self.i_marker.header.frame_id = self.base_tf
        self.i_marker.header.stamp = rospy.Time.now()
        self.i_marker.name = "user_input_marker"
        self.i_marker.description = "Input Pose"

        # attach marker to the specified frame
        if self.attach_interactive_marker_tf is not None:
            trans, rot = get_transform(self.base_tf, self.attach_interactive_marker_tf, self.listener)
            if None not in [trans, rot]:
                self.i_marker.pose.position = Point(*trans)
                self.i_marker.pose.orientation = Quaternion(*rot)
            else:
                rospy.logwarn("Cannot align interaction marker with TF {}. Set a default pose.".format(
                    self.attach_interactive_marker_tf))
                self.i_marker.pose.position = Point([1, 1, 1])
                self.i_marker.pose.orientation = Quaternion([0, 0, 0, 1])

        # create a grey box marker
        self.dot_marker.type = self.dot_marker.SPHERE
        self.dot_marker.scale = Vector3(0.05, 0.05, 0.05)
        self.dot_marker.color = ColorRGBA(0.7, 0.3, 0.3, 0.7)
        self.dot_marker.pose.position = Point(-0.2, 0.2, 0.2)

        # create a non-interactive control which contains the dot marker
        self.dot_control.always_visible = True
        self.dot_control.markers.append(self.dot_marker)
        self.dot_control.orientation_mode = InteractiveMarkerControl.FIXED
        self.dot_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        self.i_marker.controls.append(self.dot_control)

        # add the control to the interactive marker
        self.m_marker_server.insert(self.i_marker, self.calculate_inv_kin)
        self.m_marker_server.applyChanges()

    def run(self):
        rospy.spin()


class Modes:
    INVERSE_KIN = "inverse_kinematics"
    FODWARD_KIN = "forward_kinematics"
