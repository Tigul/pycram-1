from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.designators.action_designator import TransportActionDescription, SearchActionDescription
from pycram.designators.location_designator import KnowledgeLocation
from pycram.designators.object_designator import ResolutionStrategyObject
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object

from pycrap.ontologies import Robot, Milk

world = BulletWorld(WorldMode.DIRECT)

pr2 = Object("pr2", Robot, "pr2.urdf")

plan = SequentialPlan(
    TransportActionDescription(
        ResolutionStrategyObject(strategy=SearchActionDescription(KnowledgeLocation(Milk), Milk)),
    PoseStamped.from_list([4, 5, 1])),
)

with simulated_robot:
    plan.perform()