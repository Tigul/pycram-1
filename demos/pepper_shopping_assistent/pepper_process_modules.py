from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld
from pycram.helper import transform
from pycram.ik import request_ik
from pycram.helper import _transform_to_torso, _apply_ik, make_pose_stamped_msg, list2pose
from pycram.local_transformer import local_transformer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID, GoalStatus
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import actionlib
import pycram.bullet_world_reasoning as btr
import pybullet as p
import numpy as np
import time


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of Pepper and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "right":
        for joint, pose in robot_description.i.get_static_joint_chain("right", "park").items():
            robot.set_joint_state(joint, pose)
    if arm == "left":
        for joint, pose in robot_description.i.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class PepperNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            robot = BulletWorld.robot
            robot.set_position_and_orientation(solution['target'], solution['orientation'])
            time.sleep(0.5)
            local_transformer.update_from_btr()




class PepperParkArms(ProcessModule):
    """
    This process module is for moving the arms in a parking position.
    It is currently not used.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class PepperMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """
    # pan_joiunt = HeadYaw
    # tilt_joint = HeadPitch
    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'looking':
            target = solutions['target']
            robot = BulletWorld.robot
            if type(target) is str:
                target_frame = local_transformer.projection_namespace + '/' + target \
                    if local_transformer.projection_namespace \
                    else target
                target = local_transformer.tf_transform(local_transformer.map_frame, target_frame)[0]
            pose_in_pan = transform(target, robot.get_link_position("Neck"))
            pose_in_tilt = transform(target, robot.get_link_position("Head"))

            new_pan = np.arctan([pose_in_pan[1], pose_in_pan[0]])
            new_tilt = np.arctan([-pose_in_tilt[2], pose_in_tilt[0] ** 2 + pose_in_tilt[1] ** 2])

            robot.set_joint_state("HeadYaw", new_pan[0])
            robot.set_joint_state("HeadPitch", new_tilt[0])


class PepperMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-gripper":
            robot = BulletWorld.robot
            gripper = solution['gripper']
            motion = solution['motion']
            for joint, state in robot_description.i.get_static_gripper_chain(gripper, motion).items():
                robot.set_joint_state(joint, state)
            time.sleep(0.5)


class PepperMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-tcp":
            target = solution['target']
            target = _transform_to_torso([target, [0, 0, 0, 1]])
            gripper = solution['gripper']
            robot = BulletWorld.robot
            joints = ik_joints_left if gripper == "l_gripper_tool_frame" else ik_joints_right
            #inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), target)
            inv = request_ik(Pepper_root_link, gripper, target, robot, joints)
            _apply_ik(robot, inv, gripper)
            time.sleep(0.5)

class PepperMoveJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-joints":
            robot = BulletWorld.robot
            right_arm_poses = solution['right_arm_poses']
            left_arm_poses = solution['left_arm_poses']
            if type(right_arm_poses) == dict:
                for joint, pose in right_arm_poses.items():
                    robot.set_joint_state(joint, pose)
            elif type(right_arm_poses) == str and right_arm_poses == "park":
                _park_arms("right")
            elif type(right_arm_poses) == str and right_arm_poses == "point":
                for joint, value in robot_description.i.get_static_joint_chain('right', 'point').items():
                    robot.set_joint_state(joint, value)
                # for i in range(1, 4):
                #     robot.set_joint_state('RFinger1' + str(i), 1)

            if type(left_arm_poses) == dict:
                for joint, pose in left_arm_poses.items():
                    robot.set_joint_state(joint, pose)
            elif type(left_arm_poses) == str and left_arm_poses == "park":
                _park_arms("left")
            elif type(left_arm_poses) == str and left_arm_poses == "point":
                for joint, value in robot_description.i.get_static_joint_chain('left', 'point').items():
                    robot.set_joint_state(joint, value)
                # for i in range(1, 4):
                #     robot.set_joint_state('LFinger1' + str(i), 1)

            time.sleep(0.5)


class PepperWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "world-state-detecting":
            obj_type = solution['object']
            return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class PepperRealNavigation(ProcessModule):
    def _execute(self, desig):
        def active_callback():
            rospy.loginfo("Start Navigating")
        def feedback_callback(msg):
            pass
        def done_callback(state, result):
            for k in GoalStatus.__dict__.keys():
                if state == GoalStatus.__dict__[k]:
                    rospy.loginfo(f"Navigation has Finished with the result: {k}")
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            goal = MoveBaseGoal()
            pose = list2pose([solution['target'], solution['orientation']])
            goal.target_pose.pose = pose
            #goal.target_pose.pose.orientation.w = 1
            #goal.target_pose.pose.position.x = 1
            #goal.target_pose.pose.position.y = 1
            goal.target_pose.header.frame_id = "map"
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo("Waiting for action server")
            client.wait_for_server()
            client.send_goal(goal,active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
            wait = client.wait_for_result()

class PepperRealMoveJoints(ProcessModule):
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'move-joints':
            moveit_commander.roscpp_initialize(sys.argv)
            robot = moveit_commander.RobotCommander()
            scene = moveit_commander.PlanningSceneInterface()

            right_arm_poses = solution['right_arm_poses']
            left_arm_poses = solution['left_arm_poses']
            if type(right_arm_poses) == dict:
                move_group = moveit_commander.MoveGroupCommander('right_arm')
                joint_goal = right_arm_poses.values()
                move_group.go(joint_goal, wait=True)
                move_group.stop()
            elif type(right_arm_poses) == str:
                move_group = moveit_commander.MoveGroupCommander('right_arm')
                joint_goal = list(robot_description.i.get_static_joint_chain('right', right_arm_poses).values())
                move_group.go(joint_goal, wait=True)
                move_group.stop()

            if type(left_arm_poses) == dict:
                move_group = moveit_commander.MoveGroupCommander('left_arm')
                joint_goal = left_arm_poses.values()
                move_group.go(joint_goal, wait=True)
                move_group.stop()
            elif type(left_arm_poses) == str:
                move_group = moveit_commander.MoveGroupCommander('left_arm')
                joint_goal = list(robot_description.i.get_static_joint_chain('left', left_arm_poses).values())
                move_group.go(joint_goal, wait=True)
                move_group.stop()


PepperProcessModulesSimulated = {'navigate' : PepperNavigation(),
                              'looking' : PepperMoveHead(),
                              'opening_gripper' : PepperMoveGripper(),
                              'closing_gripper' : PepperMoveGripper(),
                              'move-tcp' : PepperMoveTCP(),
                              'move-joints' : PepperMoveJoints(),
                              'world-state-detecting' : PepperWorldStateDetecting()}

PepperProcessModulesReal = {'navigate': PepperRealNavigation(),
                            'move-joints': PepperRealMoveJoints()}

def available_process_modules(desig):
    robot_name = robot_description.i.name
    robot_type = ProcessModule.robot_type
    type = desig.prop_value('cmd')

    if robot_name == 'pepper':
        if robot_type == 'simulated':
            return PepperProcessModulesSimulated[type]
        elif robot_type == 'real':
            return PepperProcessModulesReal[type]
        elif robot_type == "":
            rospy.logerr(f"No robot_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
        else:
            rospy.logerr(f"No Process Module could be found for robot {robot_name}")

ProcessModule.resolvers.append(available_process_modules)
