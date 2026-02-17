from utc.src.graph import RoadNetwork, Junction, Edge, Route, Graph
from utc.src.routing.base.traffic_problem import TrafficProblem, ControlledVehicle
from utc.src.routing.pddl.base.pddl_problem import PddlProblem
from utc.src.routing.pddl.base.pddl_result import PddlResult
from utc.src.routing.pddl.generators import ProblemGenerator, ResultGenerator
from utc.src.routing.routing_options import SolverOptions
from utc.src.routing.traffic.network_builder import NetworkBuilder, NetworkBuilderOptions
from utc.src.simulator.scenario import Scenario
from typing import Optional, List, Set, Tuple, Dict
import time




class DSO:
    """
    Class dealing with centralized routing approach, i.e. other vehicles are taken into account
    """
    def __init__(self, new_scenario: Scenario, sub_graphs: List[Graph], options1: NetworkBuilderOptions):
        self.sub_graphs: List[Graph] = sub_graphs
        self.builders: List[NetworkBuilder] = [NetworkBuilder(sub_graph, options1) for sub_graph in sub_graphs]
        self.counter: int = 0
        self.problem_generator: ProblemGenerator = ProblemGenerator(new_scenario, dynamic_cost=True)
        self.result_generator: ResultGenerator = ResultGenerator()
        print(f"Successfully initialized DSO routing for: {len(self.sub_graphs)} sub-graphs")

    def route_vehicles(self, vehicles: List[ControlledVehicle]) -> Tuple[List[Optional[Route]], float]:
        """
        :param vehicles:
        :return:
        """
        assert(len(self.builders) != 0)
        now: float = time.time()
        # TODO simplify by adding PddlProblem & Result to TrafficProblem
        # Create mapping of {TrafficProblem -> (PddlProblem, PddlResult)}
        mapping: Dict[TrafficProblem: Tuple[Optional[PddlProblem], Optional[PddlResult]]] = {}
        # Construct TrafficProblem
        for traffic_problem in self.construct_traffic_problems(vehicles):
            mapping[traffic_problem] = (None, None)
        if not mapping:
            return [], 0
        # Convert TrafficProblem to PDDL
        valid_instances: List[Tuple[TrafficProblem, PddlProblem]] = []
        for traffic_problem in mapping.keys():
            pddl_problem: Optional[PddlProblem] = None
            if traffic_problem.is_valid():
                pddl_problem = self.problem_generator.generate_pddl_problem(traffic_problem, "utc_allowed")
            mapping[traffic_problem] = (pddl_problem, None)
            if pddl_problem is not None:
                valid_instances.append((traffic_problem, pddl_problem))
        if not valid_instances:
            return [], 0
        self.counter += 1
        # Call solvers with PDDL files
        pddl_results: List[Optional[PddlResult]] = self.result_generator.generate_results(
            [instance[1] for instance in valid_instances],
            "utc_allowed", "mip_allowed",
            self.problem_generator.new_scenario.scenario_dir,
            timeout=8.5, processes=2
        )
        for i, valid_instance in enumerate(valid_instances):
            mapping[valid_instance[0]] = (valid_instance[1], pddl_results[i])
        # Parse & return new routes
        routes: Dict[str, Optional[Route]] = {vehicle.id: None for vehicle in vehicles}
        for region_id, (traffic_problem, (_, pddl_result)) in enumerate(mapping.items()):
            if pddl_result is None:
                continue
            new_routes: Optional[Dict[str, Route]] = pddl_result.extract_routes(traffic_problem)
            if new_routes is None:
                continue
            for vehicle_id, route in new_routes.items():
                routes[vehicle_id] = route
        return list(routes.values()), round(time.time() - now, 3)

    def construct_traffic_problems(self, vehicles: List[ControlledVehicle]) -> List[TrafficProblem]:
        """
        :return:
        """
        problems: List[TrafficProblem] = [
            TrafficProblem(f"{self.counter}_{self.sub_graphs[i].road_network.map_name}", [])
            for i in range(len(self.builders))
        ]
        # Filter out vehicles for regions (ignore those outside)
        for vehicle in vehicles:
            region_id: int = vehicle.route.get_current_segment().region_id
            if region_id == -1:
                continue
            assert(0 <= region_id < len(self.builders))
            problems[region_id].vehicles[vehicle.id] = vehicle
        # Construct road network for each traffic problem
        for i, problem in enumerate(problems):
            if not problem.vehicles:
                continue
            self.builders[i].build_network(problem)
        return problems


