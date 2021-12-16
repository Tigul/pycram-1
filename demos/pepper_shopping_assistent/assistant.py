from pycram.knowrob import get_pose_for_product_type
from pycram.process_module import with_real_robot, real_robot, with_simulated_robot, simulated_robot
from pycram.motion_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.ik import request_ik
from navigation import navigation, nearest_wp
from std_msgs.msg import String
from pycram.robot_description import InitializedRobotDescription as robot_description
from std_srvs.srv import Empty
from pepper_behaviour_srvs.srv import Speak, MoveHead
from geometry_msgs.msg import PoseStamped
import numpy as np
import pybullet as p
import pepper_process_modules
import action_desig_grounding
import motion_desig_grounding
import rospy
import tf
import time
import moveit_commander
import moveit_msgs.msg
import sys
import json

@with_real_robot
def assistant(product_args):
    """
    The shopping assistant plan, this plan queries KnowRob for the position of
    a shelf which stores a product corresponding to the product category and
    additional preferences. After recieving the shelf pose the waypoint with the
    shortest distance is calculated and a navigation goal for this waypoint is
    sent to the ROS navigation stack. After arriving at the waypoint the robot
    points at the searched product.
    :param product_args: The argumnets containing the product category as well as
    the additional preferences
    """
    product_type = product_args['product']
    MotionDesignator(SpeechMotionDescription(text="ok, follow me")).perform()
    MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="park", right_arm_config="park")).perform()

    tf_listener = tf.TransformListener()
    time.sleep(1)
    shelf_pose = get_pose_for_product_type(product_type)
    robot_pose = tf_listener.lookupTransform('/map', '/base', rospy.Time(0))
    goal = nearest_wp(shelf_pose[0])
    robot_pose = tf_listener.lookupTransform('/map', '/base', rospy.Time(0))
    angle_to_goal = np.arctan2(goal[1] - shelf_pose[0][1], goal[0] - shelf_pose[0][0]) + np.pi
    angle_as_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes="sxyz"))
    MotionDesignator(MoveMotionDescription(target=goal, orientation=angle_as_quanternion)).perform()
    point_to(shelf_pose[0])


def app_callback(msg):
    """
    The callbackmethod for the assistant topic subscriber, this method then calls
    the actual assistant plan with the dictionary contained in the ROS message.
    :param msg: The ROS message containing the dictionary with product category
    and additional preferences.
    """
    dict = json.loads(msg.data)
    assistant(dict)

@with_real_robot
def point_to(goal_pose):
    """
    This method lets Pepper point at a specific point in space, to achieve this
    the robot first turns toward this point. Then the arm is turned so the palm
    faces the user and lastly the shoulder angle is set such that the arm aligns
    with the point.
    :param goal_pose: The position in space at which Pepper should point.
    """
    shoulder_angle = np.arctan2(goal_pose[2]-0.9 , 0.8)
    shoulder_joint = "LShoulderPitch"
    shoulder_angle = round(-shoulder_angle+0.1, 2)
    joint_goal = robot_description.i.get_static_joint_chain('left', 'point')
    joint_goal[shoulder_joint] = shoulder_angle

    MotionDesignator(MoveArmJointsMotionDescription(left_arm_poses=joint_goal)).perform()
    #p.addUserDebugLine(shoulder_pose, goal_pose)

# the ROS subscriber
rospy.Subscriber('/assistant', String, app_callback)
