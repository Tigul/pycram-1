from pycram.knowrob import get_pose_for_product_type
from pycram.process_module import with_real_robot, real_robot
from pycram.motion_designator import *
from pycram.ik import request_ik
from navigation import navigation
from std_msgs.msg import String
from pycram.robot_description import InitializedRobotDescription as robot_description
import numpy as np
import pepper_process_modules
import action_desig_grounding
import motion_desig_grounding
import rospy
import tf
import time

#node = rospy.init_node("assistant")
#rospy.Subscriber('/assistant', String, app_callback)
#rospy.spin()
# http://knowrob.org/kb/shop.owl#FruitOrCereal

type_to_knowrob = {}

@with_real_robot
def assistant(product_type):
    MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="park", right_arm_config="park")).perform()
    tf_listener = tf.TransformListener()
    time.sleep(2)
    item_pose = get_pose_for_product_type(product_type)
    robot_pose = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    route = navigation(item_pose[0], robot_pose[0])
    #route = navigation([-0.1, -3.2, 0], [0, 0, -0.70, 0.70])
    for pose, orientation in route:
        robot_pose = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        angle_to_goal = np.arctan2(pose[1] - robot_pose[0][1], pose[0] - robot_pose[0][0])
        angle_as_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes="sxyz"))
        print(angle_as_quanternion)
        MotionDesignator(MoveMotionDescription(target=pose, orientation=angle_as_quanternion)).perform()
    MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="point")).perform()

@with_real_robot
def simple_assistant():
    tf_listener = tf.TransformListener()
    time.sleep(2)
    robot_pose = tf_listener.lookupTransform('/base_footprint', '/map', rospy.Time(0))
    print(robot_pose)
    route = navigation([1.5, -3.4, 0], robot_pose[0])
    for pose, orientation in route:
        print(pose)
        print(orientation)
        MotionDesignator(MoveMotionDescription(target=pose, orientation=orientation)).perform()

#print(get_pose_for_product_type('http://knowrob.org/kb/shop.owl#FruitOrCereal'))
#assistant('http://knowrob.org/kb/shop.owl#FruitOrCereal')

def app_callback(msg):
    knowrob_type = type_to_knowrob[msg.data]
    assistant(knowrob_type)

def point_to(arm, goal):
    shoulder_link = "RShoulder" if arm == "right" else "LShoulder"
    hand_link = "RHand" if arm == "right" else "LHand"
    tf_listener = tf.TransformListener()
    time.sleep(2)

    shoulder_pose = tf_listener.lookupTransform('/map', shoulder_link, rospy.Time(0))
    print(f"tf shoulder: {shoulder_pose}")
    foot_print_pose = tf_listener.lookupTransform('/base_footprint', shoulder_link, rospy.Time(0))

    ee_goal = _calculate_ee_pose(shoulder_pose[0], goal)
    ee_goal_in_footprint = list(np.array(foot_print_pose[0]) + ee_goal)

    arm_joints = robot_description.i._safely_access_chains(arm).joints
    ik = request_ik(shoulder_link, hand_link, [ee_goal_in_footprint, [0, 0, 0, 1]], arm_joints)

    values = {}
    for joint, value in zip(arm_joints, ik):
        values[joint] = value

    description = MoveArmJointsMotionDescription()
    description.__dict__[arm+'_arm_poses'] = values

    MotionDesignator(describtion).perform()


def _calculate_ee_pose(shoulder_pose, goal_pose):
    shoulder_pose = np.array(shoulder_pose)
    goal_pose = np.array(goal_pose)

    # Vector from the shoulder to the goal
    direction_vector = goal_pose - shoulder_pose
    dir_length = np.linalg.norm(direction_vector)
    # Short vector has rigt direction and length
    short_vector = (0.4 / dir_length)*direction_vector

    vector_in_map = shoulder_pose + short_vector
    print(f"shoulder pose: {shoulder_pose}")
    print(f"short vector: {short_vector}")
    print(f"vector in map: {vector_in_map}")
    # Return vector is in shoulder frame
    return short_vector

# with real_robot:
#     MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="park", right_arm_config="park")).perform()
point_to("right", [3, 1.5, 0.9])
