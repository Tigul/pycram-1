import rospy

print("start of file")
from pycram.robot_descriptions import robot_description
print("after robot description")

from pycram.bullet_world import BulletWorld, Object, Use_shadow_world
from pycram.enums import ObjectType
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot
from pycram.bullet_world_reasoning import *
from pycram.external_interfaces.ik import request_ik

print("before world")
rospy.logwarn("before world")
world = BulletWorld("DIRECT")
print("world created")
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
print("robot created")
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
print("milk created")
print("before apartment")
apartement = Object("apartement", ObjectType.ENVIRONMENT, "kitchen.urdf")

print("start blocking test")
milk.set_pose(Pose([0.5, -0.7, 1]))
print("move milk")
robot.set_pose(Pose())
print("move robot")
time.sleep(1)
print("sleep")
#blocking(Pose([0.5, -0.7, 1]), robot, robot_description.get_tool_frame("right"))
#reachable(Pose([0.5, -0.7, 1]), robot, robot_description.get_tool_frame("right"))
shadow_robot = world.get_shadow_object(robot)
with simulated_robot:
    with Use_shadow_world():
        shadow_robot.set_pose(Pose([1,2,0]))
        shadow_robot.set_pose(Pose([0,0,0]))

        rospy.wait_for_service("/pr2_right_arm_kinematics/get_ik")

        #request_ik(Pose([0.5, -0.7, 1]), robot, robot_description.chains["right"].joints, robot_description.get_tool_frame("right"))
        #MoveTCPMotion(Pose([0.5, -0.7, 1])).resolve().perform()
print("test blocking")
print("end blocking test")

print("-" * 50)
print("done")

world.exit()
