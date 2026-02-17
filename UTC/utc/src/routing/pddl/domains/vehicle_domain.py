from utc.src.routing.base.traffic_problem import TrafficProblem
from utc.src.routing.pddl.base.pddl_problem import PddlProblem


class VehicleDomain:
    """
    Class holding representation of vehicles for '.pddl' problem files
    """
    def __init__(self):
        # Name of group used by vehicles
        self.vehicle_group_name: str = "car"

    def process_vehicles(self, pddl_problem: PddlProblem, traffic_problem: TrafficProblem) -> bool:
        """
        Adds vehicles to ':object' and adds their initial and
        destination positions to ':init' & ':goal'

        :param pddl_problem: instance of pddl problem
        :param traffic_problem: instance of traffic problem
        :return: True on success, false otherwise
        """
        for vehicle in traffic_problem.vehicles.values():
            if vehicle.id not in traffic_problem.sub_graphs:
                continue
            # Object definition
            pddl_id: str = f"v{vehicle.internal_id}"
            pddl_problem.add_object(self.vehicle_group_name, pddl_id)
            # Initial position (dynamic)
            pddl_problem.add_init_state(f"(at {pddl_id} js{vehicle.internal_id})")
            # Destination pos (static)
            pddl_problem.add_init_state(f"(togo {pddl_id} je{vehicle.internal_id})")
            # Goal position (static)
            pddl_problem.add_goal_state(f"(at {pddl_id} je{vehicle.internal_id})")
        return True
