from pycram.datastructures.pose import Pose
from pycram.designators.location_designator import CostmapLocation
from pycram.designators.object_designator import BelieveObject
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.world_concepts.world_object import Object
from pycram.process_module import simulated_robot
from pycram.ros_utils.robot_state_updater import RobotStateUpdater

from pycram.designators.action_designator import NavigateAction


world = BulletWorld(WorldMode.GUI)

apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")
hsrb = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf")


r = RobotStateUpdater("/tf", "/hsrb/joint_states")
robot_desig = BelieveObject(types=[ObjectType.ROBOT])

with simulated_robot:
    origin_pose = CostmapLocation(target=Pose([0, 0, 0]), reachable_for=robot_desig.resolve()).resolve()

    NavigateAction([origin_pose.pose]).resolve().perform()

    # Code to show a picture on the display
