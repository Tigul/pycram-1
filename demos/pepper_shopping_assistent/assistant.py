from pycram.knowrob import get_pose_for_item
from pycram.process_module import with_real_robot
from navigation import navigation
import pepper_process_modules
import rospy
import tf

node = rospy.init_node("assistant")
tf_listener = tf.TransformListener()

@with_real_robot
def assistant():
    item_pose = get_pose_for_item(product_type, item)
    robot_pose = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    print(robot_pose)
    #route = navigation(item_pose, )
