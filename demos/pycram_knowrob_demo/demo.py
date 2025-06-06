from pycram.datastructures.enums import WorldMode, Arms, TorsoState
from pycram.datastructures.pose import PoseStamped
from pycram.designators.action_designator import TransportActionDescription, SearchActionDescription, \
    ParkArmsActionDescription, MoveTorsoActionDescription
from pycram.designators.location_designator import KnowledgeLocation
from pycram.designators.object_designator import ResolutionStrategyObject
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object

from pycrap.ontologies import Robot, Milk, Apartment

world = BulletWorld(WorldMode.GUI)

pr2 = Object("pr2", Robot, "pr2.urdf", pose=PoseStamped.from_list([1, 1, 0]))
apartment = Object("apartment", Apartment, "apartment.urdf")
milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([0.5, 2.5, 1.4]))

plan = SequentialPlan(
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
    TransportActionDescription(
        ResolutionStrategyObject(strategy=SearchActionDescription(KnowledgeLocation(Milk), Milk)),
    PoseStamped.from_list([4, 5, 1])),
)

with simulated_robot:
    plan.perform()