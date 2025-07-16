from dataclasses import dataclass, field

from semantic_world.world import World
from semantic_world.world_entity import Body, Connection

from ..datastructures.pose import PoseStamped


@dataclass
class SemanticWorld:

    sem_world: World = field(default_factory=World)
    """
    Instance of the World class that represents the semantic world.
    """
    def get_body_pose(self, body: Body, relative_to: Body = None) -> PoseStamped:
        """
        Get the pose of a body in the semantic world relative to another body if specified otherwise in the world frame.
        :param body:
        :param relative_to:
        :return:
        """
        pass

    def set_body_pose(self, body: Body, pose: PoseStamped) -> None:
        """
        Set the pose of a body in the semantic world relative to another body.

        :param body: The body whose pose is to be set.
        :param pose: The new pose for the body.
        """
        pass

    def get_joint_position(self, connection: Connection) -> float:
        """
        Get the position of a joint in the semantic world.

        :param connection: The connection representing the joint.
        :return: The position of the joint.
        """
        pass

    def set_joint_position(self, connection: Connection, position: float) -> None:
        """
        Set the position of a joint in the semantic world.

        :param connection: The connection representing the joint.
        :param position: The new position for the joint.
        """
        pass

    def get_inverse_kinematics(self, body: Body, target_pose: PoseStamped) -> dict[str, float]:
        """
        Get the inverse kinematics for a body to reach a target pose.

        :param body: The body for which to compute the inverse kinematics.
        :param target_pose: The target pose to reach.
        :return: A list of joint positions that achieve the target pose.
        """
        pass

    def  transform_pose(self, pose: PoseStamped, target_frame: Body) -> PoseStamped:
        """
        Transform a pose from the world frame to the target frame.

        :param pose: The pose to transform.
        :param target_frame: The body representing the target frame.
        :return: The transformed pose in the target frame.
        """
        pass

    def add_body(self, body: Body) -> None:
        """
        Add a body to the semantic world.

        :param body: The body to add.
        """
        self.sem_world.add_body(body)

    def remove_body(self, body: Body) -> None:
        """
        Remove a body from the semantic world.
        :param body:
        :return:
        """
        self.sem_world.remove_body(body)

    def get_body_by_name(self, name: str) -> Body:
        """
        Get a body by its name from the semantic world.
        :param name: The name of the body to retrieve.
        :return: The body with the specified name.
        """
        return self.sem_world.get_body_by_name(name)

    def get_connection_by_name(self, name: str) -> Connection:
        """
        Get a connection by its name from the semantic world.
        :param name: The name of the connection to retrieve.
        :return: The connection with the specified name.
        """
        return self.sem_world.get_connection_by_name(name)

    @property
    def bodies(self):
        """
        Get all bodies in the semantic world.
        :return: A list of all bodies in the semantic world.
        """
        return self.sem_world.bodies

    @property
    def bodies_with_collisions(self):
        """
        Get all bodies in the semantic world that have collision enabled.
        :return: A list of all bodies with collision enabled.
        """
        return self.sem_world.bodies_with_collisions

    @property
    def connections(self):
        """
        Get all connections in the semantic world.
        :return: A list of all connections in the semantic world.
        """
        return self.sem_world.connections

    def save_state(self):
        """
        Save the current state of the semantic world.
        :return: The current state of the semantic world.
        """
        pass

    def restore_state(self, state):
        """
        Restore the semantic world to a previously saved state.
        :param state: The state to restore to.
        """
        pass

    def reset_world(self):
        """
        Reset the semantic world to its initial state.
        """
        pass

    def set_initial_state(self) -> None:
        """
        Set the initial state of the semantic world to the current state.
        """
        pass

    def merge_world(self, other_world: World) -> None:
        """
        Merge another semantic world into this one.

        :param other_world: The semantic world to merge into this one.
        """
        self.sem_world.merge_world(other_world)

    def add_vis_axis(self, pose: PoseStamped) -> None:
        pass

    def remove_vis_axis(self, pose: PoseStamped) -> None:
        pass


