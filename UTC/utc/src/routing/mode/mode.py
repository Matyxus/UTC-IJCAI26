from utc.src.routing.base.controlled_vehicle import ControlledVehicle
from utc.src.routing.base.traffic_problem import TrafficProblem
from utc.src.routing.routing_options import RoutingOptions
from utc.src.routing.pddl.generators import ResultGenerator, ProblemGenerator
from utc.src.routing.traffic.network_builder import NetworkBuilder
from utc.src.simulator.scenario import Scenario
from utc.src.graph import Graph, RoadNetwork
from typing import Optional, Union, List


class Mode:
    """
    Super Class for online & offline traffic routing,
    provides utility parameters and acts as an interface
    """
    def __init__(self, options: RoutingOptions):
        """
        :param options: of Pddl generation
        """
        assert(options is not None)
        self.options: RoutingOptions = options
        self.scenario: Scenario = None
        self.new_scenario: Scenario = None
        self.graph: Graph = None
        self.sub_graphs: List[Graph] = []
        self.problem_generator: ProblemGenerator = None
        self.result_generator: ResultGenerator = None
        self.network_builder: Union[NetworkBuilder, List[NetworkBuilder], None] = None
        assert(self.initialize())
        print(f"Successfully initialized {self.__class__.__name__} routing mode.")

    def initialize(self) -> bool:
        """
        :return: True on success, false otherwise.
        """
        # Initialize scenarios
        self.scenario = Scenario(self.options.init.scenario)
        self.new_scenario = Scenario(self.options.init.new_scenario, True)
        if not self.scenario.exists() or self.scenario.scenario_dir.get_config(self.options.init.config) is None:
            return False
        elif not self.new_scenario.scenario_dir.initialize_dir(pddl=True, info_dir=True):
            return False
        # Initialize graph
        self.graph: Graph = Graph(RoadNetwork())
        if not self.graph.loader.load_map(self.scenario.config_file.get_network()):
            return False
        # Initialize sub-graph(s), if there is(are) any
        for sub_graph_name in self.options.builder.regions:
            self.sub_graphs.append(Graph(RoadNetwork()))
            if not self.sub_graphs[-1].loader.load_map(sub_graph_name):
                return False
        # Initialize network builder(s), if needed
        self.network_builder = [NetworkBuilder(graph, self.options.builder) for graph in self.sub_graphs]
        # Initialize Problem & Result generators
        self.problem_generator = ProblemGenerator(self.new_scenario, self.options.init.mode.dynamic_cost)
        self.result_generator = ResultGenerator(self.new_scenario.scenario_dir)
        return True

    def run(self) -> List[ControlledVehicle]:
        """
        Main method to run vehicle routing

        :return: All re-routed vehicles, which have their new routes
        """
        raise NotImplementedError("Error, method 'run' must be implemented by children of 'Mode' class!")

    def save_results(self, problems: List[TrafficProblem]) -> None:
        """
        Saves new routes with vehicles into a new scenario.

        :return: None
        """
        # for (vehicle, route) in :
        #     route_id: str = self.new_scenario.routes_file.add_route(route, re_index=True)
        #     vehicle.attrib["route"] = route_id
        #     self.new_scenario.vehicles_file.add_vehicle(vehicle)
        # Save vehicles in new scenario
        # extractor = VehicleExtractor(self.scenario.vehicles_file, self.scenario.routes_file)
        # entry: VehicleEntry = extractor.estimate_arrival_naive((0, float("inf")))
        # for vehicle in entry.vehicles.values():
        #     vehicle = vehicle.to_xml()
        #     route: Element = deepcopy(entry.original_routes[vehicle.attrib["route"]])
        #     if vehicle.attrib["id"] in self.scheduler.queue.vehicles:
        #         route.attrib["edges"] = " ".join(self.scheduler.queue.vehicles[vehicle.attrib["id"]].route)
        #     route_id: str = self.new_scenario.routes_file.add_route(route, re_index=True)
        #     vehicle.attrib["route"] = route_id
        #     self.new_scenario.vehicles_file.add_vehicle(vehicle)
        return

