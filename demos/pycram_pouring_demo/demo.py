from pycram.bullet_world import BulletWorld, Object
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.designators.location_designator import *
from pycram.designators.motion_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.process_module import simulated_robot, real_robot

world = BulletWorld()
pr2 = Object("pr2", ObjectType.ROBOT, "pr2.urdf", Pose([2, 1, 0]))
apartment = Object("apa", ObjectType.ENVIRONMENT, "apartment.urdf")
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", Pose([2.3, 2.8, 1]))
cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup_origin.stl", Pose([2.3, 3, 1]))

with simulated_robot:
    (MoveTorsoAction([0.3]) | ParkArmsAction([Arms.BOTH]) | NavigateAction([Pose([1.5, 3, 0])])).perform()

    LookAtAction([Pose([2.3, 3, 1])]).resolve().perform()
    bowl_desig = DetectAction(BelieveObject(types=[ObjectType.BOWL])).resolve().perform()
    cup_desig = DetectAction(BelieveObject(types=[ObjectType.JEROEN_CUP])).resolve().perform()
    robot_desig = BelieveObject(types=[ObjectType.ROBOT]).resolve()

    pickup_loc = CostmapLocation(cup_desig, robot_desig).resolve()

    NavigateAction([pickup_loc.pose]).resolve().perform()

    (PickUpAction(cup_desig, [pickup_loc.reachable_arms[0]], ["front"]) + ParkArmsAction([Arms.BOTH])).perform()

    pour_loc = CostmapLocation(bowl_desig, robot_desig, reachable_arm=pickup_loc.reachable_arms[0]).resolve()

    NavigateAction([pour_loc.pose]).resolve().perform()

    pour_pose = bowl_desig.pose
    pour_pose.position.z += 0.2
    pour_pose.position.y -= 0.1
    MoveTCPMotion(pour_pose, pickup_loc.reachable_arms[0]).perform()

    MoveJointsMotion(["l_wrist_roll_joint"], [-0.5]).perform()

    MoveJointsMotion(["l_wrist_roll_joint"], [0]).perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()


