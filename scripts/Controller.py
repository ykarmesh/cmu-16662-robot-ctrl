import numpy as np
import rospy

from std_msgs.msg import Float64, Empty
from trac_ik_python.trac_ik import IK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
import tf

class Controller:
    def __init__(self):
        # initialize publishers, subscribers, tflisteners
        self.arm_pub = rospy.Publisher('/goal_dynamixel_position', JointState, queue_size=1)
        self.pan_pub = rospy.Publisher('/pan/command', Float64, queue_size=1)
        self.tilt_pub = rospy.Publisher('/tilt/command', Float64, queue_size=1)
        self.gripper_open_pub = rospy.Publisher('/gripper/open', Empty, queue_size=1)
        self.gripper_close_pub = rospy.Publisher('/gripper/close', Empty, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
        self.tf_listener = tf.TransformListener()

        # global variables
        self.current_joint_state = None
        self.current_gripper_state = None

        # global frames
        self.GRIPPER_LINK = "gripper_link"
        self.BOTTOM_PLATE_LINK = "bottom_plate"

        # other classes
        self.ik_solver = IK(self.BOTTOM_PLATE_LINK, self.GRIPPER_LINK)

        # predefined variables
        self.HOME_POS_MANIPULATOR_01 = [0.004601942375302315, -0.4218447208404541, 1.6260197162628174, -0.1426602154970169, 0.010737866163253784]
        self.HOME_POS_MANIPULATOR_02 = [0.0, 0.0, 1.22, -0.142, 0.0]
        self.HOME_POS_CAMERA_01 = [0.0, 0.698]
        self.HOME_POS_CAMERA_02 = [0.698, 0.0]
        self.IK_POSITION_TOLERANCE = 0.01
        self.IK_ORIENTATION_TOLERANCE = np.pi/9
        self.MIN_CLOSING_GAP = 0.002

    def set_camera_angles(self, pan_rad, tilt_rad):
        pan_msg = Float64()
        pan_msg.data = pan_rad
        rospy.loginfo('Going to camera pan: {} rad'.format(pan_rad))
        self.pan_pub.publish(pan_msg)

        tilt_msg = Float64()
        tilt_msg.data = tilt_rad
        rospy.loginfo('Going to camera tilt: {} rad'.format(tilt_rad))
        self.tilt_pub.publish(tilt_msg)
        rospy.sleep(4)

    def set_arm_joint_angles(self, joint_target):
        joint_state = JointState()
        joint_state.position = tuple(joint_target)
        self.arm_pub.publish(joint_state)

    def get_joint_state(self, data):
        self.current_joint_state = data.position[0:5]
        self.current_gripper_state = data.positions[7:9]

    def open_gripper(self):
        empty_msg = Empty()
        rospy.loginfo('Opening gripper')
        self.gripper_open_pub.publish(empty_msg)
        rospy.sleep(4)

    def close_gripper(self):
        empty_msg = Empty()
        rospy.loginfo('Closing gripper')
        self.gripper_close_pub.publish(empty_msg)
        rospy.sleep(4)

    # finds the inverse kinematics solution for the target pose
    def compute_ik(self, target_pose):
        """
        Parameters
        ----------
        ik_solver: trac_ik_python.trac_ik Ik object
        target_pose: type geometry_msgs/Pose
        current_joint: list with length the number of joints (i.e. 5)
        Returns
        ----------
        IK solution (a list of joint angles for target_pose)
        if found, None otherwise
        """
        result = self.ik_solver.get_ik(self.current_joint_state,
                                target_pose.position.x,
                                target_pose.position.y,
                                target_pose.position.z,
                                target_pose.orientation.x,
                                target_pose.orientation.y,
                                target_pose.orientation.z,
                                target_pose.orientation.w,
                                self.IK_POSITION_TOLERANCE,
                                self.IK_POSITION_TOLERANCE,
                                self.IK_POSITION_TOLERANCE,
                                self.IK_ORIENTATION_TOLERANCE,
                                self.IK_ORIENTATION_TOLERANCE,
                                self.IK_ORIENTATION_TOLERANCE)

        if result:
            rospy.loginfo('IK solution found')
        else:
            rospy.logerr('No IK solution found')
        return result

    # Goes to the position given by FRAME and grabs the object from the top
    def go_to_pose(self, FRAME):
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(self.BOTTOM_PLATE_LINK, FRAME, rospy.Time.now(), rospy.Duration(5.0))
                P, Q = self.tf_listener.lookupTransform(self.BOTTOM_PLATE_LINK, FRAME, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
    
        O = tf.transformations.euler_from_quaternion(Q)
        Q = tf.transformations.quaternion_from_euler(0, np.pi/2, O[2])
        
        poses = [Pose(Point(P[0], P[1], P[2]+0.20), Quaternion(Q[0], Q[1], Q[2], Q[3])),
                 Pose(Point(P[0], P[1], P[2]+0.15), Quaternion(Q[0], Q[1], Q[2], Q[3]))]

        target_joint = None
        self.open_gripper()
        for pose in poses:
            if self.current_joint_state:
                target_joint = self.compute_ik(pose)
            else:
                print("Joint State Subscriber not working")
                return False

            if target_joint:
                self.set_arm_joint_angles(target_joint)
                rospy.sleep(8)
            else:
                return False
        self.close_gripper()

    # checks if the object was grasped or not
    def check_grasp(self):
        if self.current_gripper_state:
            if(np.abs(self.current_gripper_state[0]) < self.MIN_CLOSING_GAP \
                and np.abs(self.current_gripper_state[1] < self.MIN_CLOSING_GAP)):
                return False
        else:
            print("Joint State Subscriber not working")
            return False
