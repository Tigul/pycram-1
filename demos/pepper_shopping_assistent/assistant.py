from pycram.knowrob import get_pose_for_product_type
from pycram.process_module import with_real_robot, real_robot, with_simulated_robot, simulated_robot
from pycram.motion_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.ik import request_ik
from navigation import navigation
from std_msgs.msg import String
from pycram.robot_description import InitializedRobotDescription as robot_description
from std_srvs.srv import Empty
from pepper_behaviour_srvs.srv import Speak, MoveHead
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

# http://knowrob.org/kb/shop.owl#FruitOrCereal
knowrob_prefix = "http://knowrob.org/kb/product-taxonomy.owl#"

@with_real_robot
def assistant(product_args):
    product_type = product_args['product']
    #MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="park", right_arm_config="park")).perform()
    rospy.wait_for_service('move_head')
    rospy.wait_for_service('speak')
    try:
        say("Ok, follow me")
        move_head = rospy.ServiceProxy('move_head', MoveHead)
        say = rospy.ServiceProxy("speak", Speak)
        move_head([0,0])
    except rospy.ServiceException as e:
        rospy.logerr(e)
    tf_listener = tf.TransformListener()
    time.sleep(1)
    item_pose = get_pose_for_product_type(knowrob_prefix + product_type)
    #item_pose = [[-3.1, -2, 0.8], [0, 0, 0, 1]]
    robot_pose = tf_listener.lookupTransform('/map', '/base', rospy.Time(0))
    route = navigation(item_pose[0], robot_pose[0])
    print(route)
    goal = route[-1]
    #MotionDesignator(MoveMotionDescription(target=goal, orientation=)).perform()
    robot_pose = tf_listener.lookupTransform('/map', '/base', rospy.Time(0))
    angle_to_goal = np.arctan2(goal[1] - robot_pose[0][1], goal[0] - robot_pose[0][0])
    angle_as_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes="sxyz"))
    MotionDesignator(MoveMotionDescription(target=goal, orientation=angle_as_quanternion)).perform()
    point_to_2(item_pose[0])

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

def app_callback(msg):
    dict = json.loads(msg.data)
    print(dict)
    #assistant(dict)

@with_real_robot
def point_to_2(goal_pose):
    tf_listener = tf.TransformListener()
    time.sleep(2)
    base_pose = tf_listener.lookupTransform('/map', '/Tibia',  rospy.Time(0))

    #base_pose = pepper.get_position_and_orientation()
    print(base_pose)
    angle_to_goal = np.arctan2(goal_pose[1] - base_pose[0][1], goal_pose[0] - base_pose[0][0])
    angle_as_quanternion = list(tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes="sxyz"))
    #MotionDesignator(MoveMotionDescription(target=base_pose[0], orientation=angle_as_quanternion)).perform()
    time.sleep(1)
    #MotionDesignator(MoveArmJointsMotionDescription(left_arm_config='point')).perform()

    shoulder_link = "LShoulder"
    #shoulder_pose = pepper.get_link_position(shoulder_link)
    shoulder_pose = tf_listener.lookupTransform('/map', shoulder_link, rospy.Time(0))[0]
    shoulder_angle = np.arctan2(goal_pose[2] - shoulder_pose[2], goal_pose[0]-shoulder_pose[0])
    shoulder_joint = "LShoulderPitch"
    shoulder_angle = round(-shoulder_angle + 0.1, 2)
    joint_goal = robot_description.i.get_static_joint_chain('left', 'point')
    joint_goal[shoulder_joint] = shoulder_angle

    MotionDesignator(MoveArmJointsMotionDescription(left_arm_poses=joint_goal)).perform()
    #p.addUserDebugLine(shoulder_pose, goal_pose)

def correct_head_position():
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_group = moveit_commander.MoveGroupCommander('head')
    joint_names = move_group.get_active_joints()
    curr_joint_values = move_group.get_current_joint_values()
    print(joint_names)
    print(curr_joint_values)
    joint_goal = [0.0, 0.0]
    move_group.go(joint_goal, wait=True)
    move_group.stop()


# with real_robot:
#     MotionDesignator(MoveArmJointsMotionDescription(left_arm_config="park", right_arm_config="park")).perform()
#node = rospy.init_node("assistant")
rospy.Subscriber('/assistant', String, app_callback)
rospy.spin()
