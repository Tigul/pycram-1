import rospy

from pycram.datastructures.pose import Pose
from pycram.designators.location_designator import CostmapLocation
from pycram.designators.object_designator import BelieveObject
from pycram.ros.logging import loginfo
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.world_concepts.world_object import Object
from pycram.process_module import simulated_robot, real_robot
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.ros.publisher import create_publisher
from std_msgs.msg import Int32

from pycram.designators.action_designator import NavigateAction


world = BulletWorld(WorldMode.GUI)

# apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")
hsrb = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf")


r = RobotStateUpdater("/tf", "/hsrb/joint_states")
robot_desig = BelieveObject(types=[ObjectType.ROBOT])

with real_robot:
    # origin_pose = CostmapLocation(target=Pose([0, 0, 0]), reachable_for=robot_desig.resolve()).resolve()

    NavigateAction([Pose([0, 0, 0])]).resolve().perform()

    # Code to show a picture on the display
    loginfo("Publishing to image topic")
    # image_pub = create_publisher("/media_switch_topic", Int32)
    image_pub = rospy.Publisher("/media_switch_topic", Int32, queue_size=10, latch=True)
    msg = Int32()
    msg.data = 38
    image_pub.publish(msg)
