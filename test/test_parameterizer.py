from random_events.product_algebra import SimpleEvent, Event

from pycram.datastructures.enums import TorsoState
from pycram.language import SequentialPlan
from pycram.parameterizer import Parameterizer
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, NavigateActionDescription
from pycram.testing import ApartmentWorldTestCase


class AlgebraTest(ApartmentWorldTestCase):

    def test_algebra(self):
        sp = SequentialPlan(self.context,
                            MoveTorsoActionDescription(None),
                            NavigateActionDescription(None),
                            MoveTorsoActionDescription(None))

        p = Parameterizer(sp)
        distribution = p.create_fully_factorized_distribution()

        conditions = []
        for state in TorsoState:
            v1 = p.get_variable("MoveTorsoAction_0.torso_state")
            v2 = p.get_variable("MoveTorsoAction_2.torso_state")
            se = SimpleEvent({v1: state, v2: state})
            conditions.append(se)

        condition = Event(*conditions)
        condition.fill_missing_variables(p.variables)

        navigate_condition = {
            p.get_variable("NavigateAction_1.target_location.pose.position.z"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.x"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.y"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.z"): 0,
            p.get_variable("NavigateAction_1.target_location.pose.orientation.w"): 1
        }

        distribution, _ = distribution.conditional(navigate_condition)

        condition &= p.create_restrictions().as_composite_set()

        conditional, p_c = distribution.truncated(condition)

        for i in range(10):
            sample = conditional.sample(1)

            resolved = p.plan_from_sample(conditional, sample[0], self.world)
            with simulated_robot:
                resolved.perform()