from pycram.bullet_world import BulletWorld, Object, Use_shadow_world
from pycram.enums import ObjectType
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot

world = BulletWorld("DIRECT")
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
apartement = Object("apartement", ObjectType.ENVIRONMENT, "apartment.urdf")

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()