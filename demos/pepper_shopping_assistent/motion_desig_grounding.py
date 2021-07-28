from pycram.motion_designator import *

def ground_moving(self):
    if not self.orientation:
        self.orientation = [0, 0, 0, 1]
    return self.__dict__

def ground_move_arm(self):
    if self.left_arm_config or self.right_arm_config:
        self.left_arm_poses = self.left_arm_config
        self.right_arm_poses = self.right_arm_config
        return self.__dict__
    if self.right_arm_poses or self.left_arm_poses:
        return self.__dict__

MoveMotionDescription.ground = ground_moving
MoveArmJointsMotionDescription.ground = ground_move_arm


def call_ground(desig):
    return [desig._description.ground()]

MotionDesignator.resolvers.append(call_ground)
