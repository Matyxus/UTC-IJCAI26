from utc.src.routing.base.traffic_problem import TrafficProblem
from utc.src.routing.pddl.base.pddl_problem import PddlProblem
from utc.src.routing.pddl.domains.network_domain import NetworkDomain
from utc.src.routing.pddl.domains.vehicle_domain import VehicleDomain
from utc.src.simulator.scenario import Scenario
from typing import Optional


class ProblemGenerator:
    """
    Class handling the generation (conversion) of pddl problem files from traffic problems
    """
    def __init__(self, new_scenario: Scenario, dynamic_cost: bool = False):
        """
        :param new_scenario: new scenario in which pddl problems are saved
        :param dynamic_cost: dynamic cost flag
        """
        assert(new_scenario is not None and new_scenario.scenario_dir.is_loaded())
        self.new_scenario: Scenario = new_scenario
        self.network_domain: NetworkDomain = NetworkDomain(dynamic_cost)
        self.vehicle_domain: VehicleDomain = VehicleDomain()

    def generate_pddl_problem(self, problem: TrafficProblem, domain: str) -> Optional[PddlProblem]:
        """
        :param problem: the traffic problem from which PDDL is generated
        :param domain: of the pddl problem
        :return: PddlProblem class if successful, None otherwise
        """
        if problem is None or not problem.is_valid():
            print(f"Error, cannot generate pddl problem, invalid traffic instance!")
            return None
        pddl_problem: PddlProblem = PddlProblem(f"problem_{problem.info.name}", domain)
        network_success: bool = self.network_domain.process_graph(pddl_problem, problem)
        vehicle_success: bool = self.vehicle_domain.process_vehicles(pddl_problem, problem)
        if not (network_success and vehicle_success):
            print(f"Error, cannot generate pddl problem: {pddl_problem.name} !")
            return None
        pddl_problem.save(self.new_scenario.scenario_dir.problems.format_file(pddl_problem.name))
        return pddl_problem
