from pycram.knowrob import get_pose_for_product_type
from pycram.process_module import with_real_robot
from pycram.motion_designator import *
from navigation import navigation
from std_msgs.msg import String
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
assistant('http://knowrob.org/kb/shop.owl#FruitOrCereal')

def app_callback(msg):
    knowrob_type = type_to_knowrob[msg.data]
    assistant(knowrob_type)
