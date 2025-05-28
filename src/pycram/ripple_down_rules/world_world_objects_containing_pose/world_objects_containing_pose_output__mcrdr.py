from ripple_down_rules.datastructures.case import Case, create_case
from typing_extensions import Set, Union
from ripple_down_rules.utils import make_set
from .world_objects_containing_pose_output__mcrdr_defs import *
from ripple_down_rules.rdr import MultiClassRDR


attribute_name = 'output_'
conclusion_type = (set, PhysicalBody, list,)
type_ = MultiClassRDR


def classify(case: Dict) -> Set[PhysicalBody]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)
    conclusions = set()

    if conditions_337033937356385007460072279108940985389(case):
        conclusions.update(make_set(conclusion_337033937356385007460072279108940985389(case)))
    return conclusions
