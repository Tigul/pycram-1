from semantic_world.connections import Connection6DoF, Has1DOFState
from semantic_world.world import World
from semantic_world.world_entity import Body

from ..datastructures.pose import PoseStamped
from ..tf_transformations import rotation_from_matrix


class SemanticWorld(World):
    """
    A semantic world that extends the basic World class with additional functionality.
    This class can be used to create and manage a semantic representation of a physical world.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.vis_axis = []
        # Additional initialization for semantic world can be added here

    def get_body_pose(self, body: Body, relative_to: Body = None) -> PoseStamped:
        """
        Get the pose of a body in the world, relative to another body or the root.

        :param body: The body for which to get the pose.
        :param relative_to: The body to which the pose should be relative. If None, the pose is relative to the root.
        :return: A PoseStamped object representing the pose of the body.
        """
        pose = self.compute_forward_kinematics_np(body, relative_to or self.root)
        quat = rotation_from_matrix(pose)
        return PoseStamped.from_list(list(pose[:3, 3]), quat, frame=relative_to.name.name if relative_to else self.root.name.name)

    def set_body_pose(self, body: Body, pose: PoseStamped):
        """
        Set the pose of a body in the world.

        :param body: The body to set the pose for.
        :param pose: The new pose to set.
        """
        connection = self.get_connection(self.root, body)
        if isinstance(connection, Connection6DoF):
            connection.origin = pose.to_matrix()
        else:
            raise TypeError(f"Connection type {type(connection)} is not supported for setting pose.")

    def get_pose_relative_to(self, pose: PoseStamped, relative_to: Body) -> PoseStamped:
        """
        Get the pose of a PoseStamped object relative to another body.

        :param pose: The PoseStamped object to transform.
        :param relative_to: The body to which the pose should be relative.
        :return: A new PoseStamped object representing the pose relative to the specified body.
        """
        new_pose = self.compute_relative_pose(pose, relative_to, self.get_body_by_name(pose.frame_id))
        return PoseStamped.from_matrix(new_pose, frame=relative_to.name.name)

    def get_body_by_id(self, body_id: int) -> Body:
        """
        Get a body by its ID. This method can be extended to include more complex logic for retrieving bodies.

        :param body_id: The ID of the body to retrieve.
        :return: The body with the specified ID.
        """
        bodies = list(filter(lambda b: b.id == body_id, self.bodies))
        if not bodies:
            raise ValueError(f"No body found with ID {body_id}.")
        if len(bodies) > 1:
            raise ValueError(f"Multiple bodies found with ID {body_id}. That should not happen.")
        return bodies[0]

    def get_joint_position(self, joint_name: str) -> float:
        """
        Get the position of a joint by its name.

        :param joint_name: The name of the joint to retrieve.
        :return: The position of the joint.
        """
        connection = self.get_connection_by_name(joint_name)
        if connection is None:
            raise ValueError(f"No joint found with name {joint_name}.")
        if isinstance(connection, Has1DOFState):
            return connection.position
        raise TypeError(f"Connection type {type(connection)} does not support joint position retrieval.")

    def set_joint_position(self, joint_name: str, position: float):
        """
        Set the position of a joint by its name.

        :param joint_name: The name of the joint to set.
        :param position: The new position to set for the joint.
        """
        connection = self.get_connection_by_name(joint_name)
        if connection is None:
            raise ValueError(f"No joint found with name {joint_name}.")
        if isinstance(connection, Has1DOFState):
            connection.position = position
        else:
            raise TypeError(f"Connection type {type(connection)} does not support setting joint position.")

    def save_state(self):
        """
        Save the current state of the world. This can be extended to include more complex state management.
        """
        pass

    def load_state(self):
        """
        Load a previously saved state of the world. This can be extended to include more complex state management.
        """
        pass

    def copy(self):
        pass

    def reset_world(self):
        pass

    def update_original_state(self):
        pass

    def add_vis_axis(self, pose: PoseStamped):
        """
        Add a visual axis to the world for debugging or visualization purposes.

        :param pose: The pose of the visual axis to add.
        """
        # TODO Transform to map frame
        self.vis_axis.append(pose)