from pycram.knowrob import get_pose_for_item
from pycram.process_module import with_real_robot
from pycram.motion_designator import MotionDesignator, MoveMotionDescription
from navigation import navigation
import pepper_process_modules
import action_desig_grounding
import motion_desig_grounding
import rospy
import tf
import time

#node = rospy.init_node("assistant")


@with_real_robot
def assistant(product_type):
    item_pose = get_pose_for_product_type(product_type)
    robot_pose = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    print(robot_pose)
    route = navigation(item_pose, robot_pose)
    for pose, orientation in route:
        print(pose)
        print(orientation)
        MotionDesignator(MoveMotionDescription(target=pose, orientation=orientation)).perform()
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
simple_assistant()
