import sys
import os
sys.path.append(os.getcwd() + "/../../src/")
import macropy
import pycram
from pycram.bullet_world import BulletWorld, Object
import pycram.bullet_world_reasoning as btr
import tf
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from pycram.designators.motion_designator import MotionDesignatorDescription, MoveArmJointsMotion
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.language import macros, par
from pycram.designators.location_designator import *
from pycram.designators.action_designator import *
from pycram.resolver.plans import Arms
from pycram.enums import Arms



world = BulletWorld()

# spawn apartment
apartment = Object("kitchen", "environment", "kitchen.urdf")
apartment_desig = ObjectDesignatorDescription(names=['kitchen']).resolve()

# spawn pr2
pr2 = Object("pr2", "robot", "pr2.urdf", position=[1.2, 2.5, 0])
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

# spawn Milkbox
milk = Object("milk", "milk", "milk.stl", position=[2.5, 2.5, 1])
milk_desig = ObjectDesignatorDescription(names=["milk"])
print("milk", milk.original_pose[0])

# spawn bowl
bowl = Object("bowl", "bowl", "bowl.stl", position=[2.5, 2.8, 0.98])
bowl_desig = ObjectDesignatorDescription(names=["bowl"])

# spawn SM_CokeBottle
SM_CokeBottle = Object("SM_CokeBottle", "SM_CokeBottle", "SM_CokeBottle.stl", position=[2.5, 3, 0.95])
SM_CokeBottle_desig = ObjectDesignatorDescription(names=["SM_CokeBottle"])

# def reset_plan():
#     ParkArmsAction([Arms.BOTH]).resolve().perform()
#     NavigateAction(target_locations=[pr2.original_pose[0]]).resolve().perform()
    
# pouring plan begins
def pouring_plan(source_obj, source_obj_desig, destination_obj, destination_obj_desig, pouring_angle):
    
    # if source_obj_name == "milk" :
    #     source_obj = milk
    #     source_obj_desig = milk_desig
    # 
    # if destination_obj_name == "bowl":
    #     destination_obj = bowl
    #     destination_obj_desig = bowl_desig
    #     
    # if source_obj_name == "cokebottle":
    #     source_obj = SM_CokeBottle
    #     source_obj_desig = SM_CokeBottle_desig
    
    # pouring_angle = float(pouring_angle)
            
    with simulated_robot:
        print("source obj location ", source_obj.original_pose[0])
        print("destination obj location ", destination_obj.original_pose[0])
        
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.3]).resolve().perform()
        pickup_pose = CostmapLocation(target=source_obj.original_pose, reachable_for=robot_desig).resolve()
        print("pickup pose: ", pickup_pose)
        pickup_arm = pickup_pose.reachable_arms[0]
        print('pickup_arm', pickup_arm)
        print("Navigate to pickup pose")
        NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()
        
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        print("Perform pickup action")
        PickUpAction(object_designator_description=source_obj_desig, arms=[pickup_arm], grasps=["front"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # do pouring by tilting, and accept time interval and speed as well
        quaternion = tf.transformations.quaternion_from_euler(pouring_angle, 0, 0, axes="sxyz")
        print('perform quaternion', quaternion)

        # tilting_pose = SemanticCostmapLocation.Location(pose=[[2.52, 2.86, 1.1], quaternion])
        # revert_tilting_pose = SemanticCostmapLocation.Location(pose=[[2.52, 2.86, 1.1], [0.0, 0, 0, 1]])
        tilting_pose = SemanticCostmapLocation.Location(pose=[destination_obj.original_pose[0], quaternion])
        print('tilting pose: ', tilting_pose)
        revert_tilting_pose = SemanticCostmapLocation.Location(pose=[destination_obj.original_pose[0], [0.0, 0, 0, 1]])
        print('revert tilting pose: ', revert_tilting_pose)
        print('perform pouring action')
        PourAction(source_obj_desig, pouring_location=[tilting_pose.pose], revert_location=[revert_tilting_pose.pose],
                   arms=[pickup_arm], wait_duration= 5).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        place_pose = SemanticCostmapLocation.Location(pose=[source_obj.original_pose[0], [0.0, 0, 0, 1.0]])
        print('place pose: ', place_pose)
        print('perform place action')
        PlaceAction(source_obj_desig, target_locations=[place_pose.pose], arms=[pickup_arm]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        final_pose = SemanticCostmapLocation.Location(pose=[[1, 2.5, 0], [0.0, 0, 0, 1.0]])
        print('final pose: ', final_pose)
        # end_pose = CostmapLocation(target=robot_desig.resolve(), reachable_for=robot_desig).resolve()
        NavigateAction(target_locations=[final_pose.pose]).resolve().perform()



def move_plan(x, y, z, yaw):
    with simulated_robot:
        # new_pose[0] = pr2.original_pose[0][0] + 1
        # new_pose[1] = pr2.original_pose[0][1]
        # new_pose[2] = pr2.original_pose[0][2]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw, axes="sxyz")
        print("moving pr2 to the pose: ", x, y, z)
        print("moving pr2 to the orientation: ", quaternion)
        
        NavigateAction(target_locations=[[[x,y,z], quaternion]]).resolve().perform()