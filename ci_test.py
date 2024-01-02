from pycram.bullet_world import BulletWorld, Object, Use_shadow_world
from pycram.enums import ObjectType
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot
from pycram.bullet_world_reasoning import *

world = BulletWorld("DIRECT")
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
#apartement = Object("apartement", ObjectType.ENVIRONMENT, "apartment.urdf")

print("start blocking test")
milk.set_pose(Pose([0.5, -0.7, 1]))
print("move milk")
robot.set_pose(Pose())
print("move robot")
time.sleep(1)
print("sleep")
blocking(Pose([0.5, -0.7, 1]), robot, robot_description.get_tool_frame("right"))
print("test blocking")
print("end blocking test")

print("-" * 50)
print("done")

world.exit()
