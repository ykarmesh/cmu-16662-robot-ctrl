#!/usr/bin/env python
"""
Example for commanding robot without moveit
"""

import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from trac_ik_python.trac_ik import IK


GRIPPER_LINK = "gripper_link"
ARM_BASE_LINK = "bottom_plate"
MOVE_GROUP_NAME = 'arm'

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'
ROSTOPIC_OPEN_GRIPPER = '/gripper/open'
ROSTOPIC_CLOSE_GRIPPER = '/gripper/close'

IK_POSITION_TOLERANCE = 0.01
IK_ORIENTATION_TOLERANCE = np.pi/9
current_joint_state = None
MIN_CLOSING_GAP = 0.002


def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)

def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)

def get_joint_state(data):
    global current_joint_state
    global current_gripper_state
    # if len(data.position) == 9:
    current_gripper_state = data.position[7:9]
    current_joint_state = data.position[0:5]

def open_gripper(pub):
    empty_msg = Empty()
    rospy.loginfo('Opening gripper')
    pub.publish(empty_msg)

def close_gripper(pub):
    empty_msg = Empty()
    rospy.loginfo('Closing gripper')
    pub.publish(empty_msg)


def compute_ik(ik_solver, target_pose, current_joint):
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
    result = ik_solver.get_ik(current_joint,
                              target_pose.position.x,
                              target_pose.position.y,
                              target_pose.position.z,
                              target_pose.orientation.x,
                              target_pose.orientation.y,
                              target_pose.orientation.z,
                              target_pose.orientation.w,
                              IK_POSITION_TOLERANCE,
                              IK_POSITION_TOLERANCE,
                              IK_POSITION_TOLERANCE,
                              IK_ORIENTATION_TOLERANCE,
                              IK_ORIENTATION_TOLERANCE,
                              IK_ORIENTATION_TOLERANCE)

    if result:
        rospy.loginfo('IK solution found')
    else:
        rospy.logerr('No IK solution found')
    return result

def main():
    rospy.init_node('IK_Control', anonymous=True)
    global current_joint_state

    rad_from_deg = np.pi/180.
    deg_from_rad = 180./np.pi

    ik_solver = IK(ARM_BASE_LINK, GRIPPER_LINK)

    # Describing the publisher subscribers
    rospy.Subscriber('/joint_states', JointState, get_joint_state)

    arm_pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT, JointState, queue_size=1)
    gripper_open_pub = rospy.Publisher(ROSTOPIC_OPEN_GRIPPER, Empty, queue_size=1)
    gripper_close_pub = rospy.Publisher(ROSTOPIC_CLOSE_GRIPPER, Empty, queue_size=1)
    rospy.sleep(2)

    # Homing of all servos
    home_joint_1 = [0.004601942375302315, -0.4218447208404541, 1.6260197162628174, -0.1426602154970169, 0.010737866163253784]
    home_joint_2 = [0.0, -1.0218447208404541, 0.5, 0.626602154970169, 0.0]
    home_joint_3 = [0.0, -1.0218447208404541, 1.1, 0.626602154970169, 0.0]

    # home_joint = [0.0, 0.0, 1.22, -0.142, 0.0]
    set_arm_joint(arm_pub, home_joint_1)
    rospy.sleep(6)

    set_arm_joint(arm_pub, home_joint_2)

    # close_gripper(gripper_close_pub)
    rospy.sleep(6)

    ## TODO: This wont work currently
    poses = [Pose(Point(0.3, 0.0, 0.45), Quaternion(0, 0, 0, 1)),
             Pose(Point(0.3, 0.0, 0.38), Quaternion(0, 0, 0, 1)),
            #  Pose(Point(0.3, 0.0, 0.45), Quaternion(0, 0, 0, 1)),
            #  Pose(Point(0.3, 0.0, 0.38), Quaternion(0, 0, 0, 1)),
            #  Pose(Point(0.3, 0.0, 0.45), Quaternion(0, 0, 0, 1)),
            #  Pose(Point(0.3, 0.0, 0.38), Quaternion(0, 0, 0, 1)),
            #  Pose(Point(0.3, 0.0, 0.45), Quaternion(0, 0, 0, 1)),
            #  Pose(Point(0.3, 0.0, 0.38), Quaternion(0, 0, 0, 1)),
            #  Pose(Point(0.3, 0.0, 0.45), Quaternion(0, 0, 0, 1)),
            #  Pose(Point(0.3, 0.0, 0.38), Quaternion(0, 0, 0, 1)),
             Pose(Point(0.3, 0.0, 0.45), Quaternion(0, 0, 0, 1))]

    target_joint = None
    # home_arm(arm_pub)
    for pose in poses:
        print(current_joint_state)
        print("Reaching a joint state")
        if current_joint_state:
            target_joint = compute_ik(
            ik_solver, pose, current_joint_state)

        if target_joint:
            set_arm_joint(arm_pub, target_joint)
            rospy.sleep(8)

    # home_arm(arm_pub)
    # open_gripper(gripper_open_pub)

    set_arm_joint(arm_pub, home_joint_3)

    # close_gripper(gripper_close_pub)
    rospy.sleep(8)
    set_arm_joint(arm_pub, home_joint_1)


if __name__ == "__main__":
    main()

    # open_gripper(gripper_open_pub)
    # rospy.sleep(4)
    # close_gripper(gripper_close_pub)
    # rospy.sleep(4)
