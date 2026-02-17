from utc.src.constants.file_system.file_types.json_file import JsonFile
from utc.src.constants.file_system.file_types.sumo_config_file import SumoConfigFile
from utc.src.routing.pddl.info.pddl_info import PddlInfo
from utc.src.routing.mode.mode import Mode
from utc.src.routing.routing_options import RoutingOptions, asdict
from utc.src.routing.mode import Online
from utc.src.routing.pddl.pddl_episode import PddlEpisode, PddlProblem, PddlResult
from utc.src.routing.mode.scheduler import  Scheduler, SumoVehicle, SumoRoute, VehicleStats
from utc.src.simulator.scenario import Scenario
from utc.src.simulator.simulation import Simulation
from utc.src.utils.vehicle_extractor import VehicleExtractor, VehicleEntry
from utc.src.graph import Junction, Route, Graph, RoadNetwork, Edge
from xml.etree.ElementTree import Element
from copy import deepcopy
import time
import traci
import heapq
from collections import defaultdict, deque
from typing import Optional, List, Set, Tuple, Dict



class DUO:
    def __init__(self, options: RoutingOptions):
        self.options = options
        self.scenario: Optional[Scenario] = None
        self.graph: Optional[Graph] = None
        self.new_scenario: Optional[Scenario] = None
        self.sub_graphs: List[Graph] = []
        assert(self.initialize())
        self.scheduler: Scheduler = Scheduler(
            [graph.road_network for graph in self.sub_graphs],
            (
                self.options.init.mode.interval[0],
                self.options.solver.timeout,
                self.options.init.mode.interval[1]
            )
        )

    def initialize(self) -> bool:
        # Initialize scenarios
        self.scenario = Scenario(self.options.init.scenario)
        self.new_scenario = Scenario(self.options.init.new_scenario, True)
        if not self.new_scenario.scenario_dir.initialize_dir(info_dir=True):
            return False
        elif not self.scenario.exists() or self.scenario.scenario_dir.get_config(self.options.init.config) is None:
            return False
        # Initialize graph
        self.graph: Graph = Graph(RoadNetwork())
        if not self.graph.loader.load_map(self.scenario.config_file.get_network()):
            return False
        # Assign starting travel time to edges (free-flow travel time)
        for edge in self.graph.road_network.get_edge_list():
            edge.attributes["travelTime"] = edge.get_travel_time()
        # Initialize sub-graph, if there is any
        for sub_graph_name in self.options.builder.regions:
            self.sub_graphs.append(Graph(RoadNetwork()))
            if not self.sub_graphs[-1].loader.load_map(sub_graph_name):
                return False
        return True

    def main(self):
        step_length: float = self.scenario.config_file.get_step_length()
        steps: int = int(self.options.init.mode.window / step_length)
        planning_steps: int = int(self.options.solver.timeout / step_length)
        assert (planning_steps < steps)
        counter, time_steps = 0, 0
        planned_vehicles: Dict[Element, Element] = {}
        total: VehicleStats = VehicleStats()
        with Simulation(self.scenario.scenario_dir.get_config(self.options.init.config), {"-W": ""}) as simulation:
            while simulation.is_running():
                print(f"---------- Current time: {simulation.get_time(False)}, step: {counter} ----------")
                # ---------------------- Advance ----------------------
                # Advance simulation by maximal timeout for solvers
                for planning_step in range(1, planning_steps + 1):
                    simulation.step()
                    self.scheduler.step(simulation)
                    # Check if results were generated sooner, assign new routes if possible asap
                    if planning_step == time_steps and planned_vehicles:
                        # Check for results in planner, check which vehicles can have routes assigned to them
                        can_assign: List[str] = self.scheduler.assign_planned(
                            [vehicle.attrib["id"] for vehicle in planned_vehicles],
                            self.graph.road_network
                        )
                        self.assign_routes(planned_vehicles, set(can_assign))
                # Check if simulation is still running
                if not simulation.is_running():
                    break
                # Continue simulation to another break point (window)
                for _ in range(steps - planning_steps):
                    simulation.step()
                    self.scheduler.step(simulation)
                # Check if simulation is still running
                if not simulation.is_running():
                    break
                # ---------------------- Schedule ----------------------
                self.scheduler.update_travel_time(self.graph.road_network, self.options.init.mode.routed_eta)
                scheduled: List[SumoVehicle] = self.scheduler.compute_etas(self.graph.road_network)
                total += self.scheduler.stats
                self.scheduler.stats.reset()
                # Generate problems & results, pretend this is asynchronous, so results will be ready next step
                if scheduled:
                    now: float = time.time()
                    planned_vehicles: Dict[Element, Element] = {}
                    for sumo_vehicle in scheduled:
                        segment: SumoRoute = sumo_vehicle.get_current_segment()
                        route_id: str = sumo_vehicle.get_attribute("route")
                        region_id: int = segment.region_id
                        # print(f"Routing vehicle: {sumo_vehicle.id}, segment: {segment.region_segment}")
                        planned_route: Route = self.dijkstra(segment.region_segment[0], segment.region_segment[-1], region_id)
                        assert(planned_route is not None)
                        planned_vehicles[deepcopy(sumo_vehicle.to_xml())] = Element(
                            "route",
                            {"id": route_id, "edges": " ".join([edge.id for edge in planned_route.edge_list])}
                        )
                        # print(f"Vehicle: {sumo_vehicle.id}, segment: {segment.region_segment}, routed: {[edge.id for edge in planned_route.edge_list]}")
                    # We are guaranteed to solve results, since we kill processes with timeout
                    time_steps = min(self.get_steps(round(time.time() - now, 1), step_length), planning_steps)
                    print(f"Generating episodes took: {round(time.time() - now, 3)}[s], steps: {time_steps}")
                else:  # Clear previous
                    time_steps = 0
                    planned_vehicles = {}
                # Continue to next step
                counter += 1
        # Safe routing information about vehicles
        with open(self.new_scenario.scenario_dir.info.format_file("scheduled.txt"), "w+") as f:
            f.write(str(total))
        # Save vehicles in new scenario
        extractor = VehicleExtractor(self.scenario.vehicles_file, self.scenario.routes_file)
        entry: VehicleEntry = extractor.estimate_arrival_naive((0, float("inf")))
        for vehicle in entry.vehicles.values():
            vehicle = vehicle.to_xml()
            route: Element = deepcopy(entry.original_routes[vehicle.attrib["route"]])
            if vehicle.attrib["id"] in self.scheduler.queue.vehicles:
                route.attrib["edges"] = " ".join(self.scheduler.queue.vehicles[vehicle.attrib["id"]].route)
            route_id: str = self.new_scenario.routes_file.add_route(route, re_index=True)
            vehicle.attrib["route"] = route_id
            self.new_scenario.vehicles_file.add_vehicle(vehicle)
        return True

    def dijkstra(self, in_edge: str, out_edge: str, region_id: int) -> Optional[Route]:
        """
        :param in_edge: Starting edge id
        :param out_edge: Ending edge id
        :return: Shortest path from 'in_edge' to 'out_edge'
        """
        # --- Init ---
        start_edge, exit_edge = self.graph.road_network.get_edges([in_edge, out_edge])
        start_route, exit_route = self.graph.road_network.get_routes([start_edge.internal_id, exit_edge.internal_id])
        assert(start_route.edge_list[0] == start_edge and exit_route.edge_list[0] == exit_edge)
        costs: Dict[str, float] = {route.id: float("inf") for route in self.graph.road_network.get_routes_list()}
        prev: Dict[str, str] = {}
        queue: List[Tuple[float, Junction, Route]] = []
        costs[start_route.id] = 0
        for out_route in self.graph.road_network.get_junction(start_route.get_destination()).travel(start_route):
            if out_route.edge_list[0].id not in self.sub_graphs[region_id].road_network.edges:
                continue
            cost, dest = out_route.travel()
            prev[out_route.id] = start_route.id
            costs[out_route.id] = cost
            heapq.heappush(queue, (cost, self.graph.road_network.get_junction(dest), out_route))
        # --- Path finding ---
        while queue:
            total_cost, junction, in_route = heapq.heappop(queue)
            # Found target, reconstruct route
            if in_route.id == exit_route.id:
                path: List[str] = [exit_route.id]
                while path[-1] in prev and costs[path[-1]] != float("inf"):
                    path.append(prev[path[-1]])
                path.reverse()
                edges = []
                for route_id in path:
                    edges += self.graph.road_network.get_route(route_id).edge_list
                if edges[1:-1]:
                    assert(self.sub_graphs[region_id].road_network.check_edge_sequence(edges[1:-1]))
                assert(self.graph.road_network.check_edge_sequence(edges))
                # print(f"Found path with cost: {total_cost} - {[edge.id for edge in edges]}")
                return Route(edges)
            elif in_route.edge_list[0].id not in self.sub_graphs[region_id].road_network.edges:
                continue
            for out_route in junction.travel(in_route):
                cost, dest = out_route.travel()
                cost += total_cost
                if cost < costs[out_route.id]:
                    prev[out_route.id] = in_route.id
                    costs[out_route.id] = cost
                    heapq.heappush(queue, (cost, self.graph.road_network.get_junction(dest), out_route))
        print(f"Unable to find path between: {in_edge, out_edge} !")
        return None

    # -------------------------------------------- Utils --------------------------------------------

    def assign_routes(self, planned_vehicles: Dict[Element, Element], allowed: Set[str]) -> bool:
        """
        :param planned_vehicles: to which route is assigned
        :param allowed: route
        :return: True on success, False otherwise
        """
        if not allowed or not planned_vehicles:
            return True
        # Pre-process
        all_vehicles: List[Tuple[Element, Element]] = []
        sumo_vehicles: List[SumoVehicle] = []
        for vehicle, route in planned_vehicles.items():
            if vehicle.attrib["id"] in allowed:
                all_vehicles.append((vehicle, route))
                sumo_vehicles.append(self.scheduler.queue.vehicles[vehicle.attrib["id"]])
        assert(len(all_vehicles) == len(sumo_vehicles))
        # Assign vehicles
        segment_index: int = 0
        for sumo_vehicle, (vehicle, route) in zip(sumo_vehicles, all_vehicles):
            # print(f"Assigning route to vehicle: {sumo_vehicle.id}")
            if sumo_vehicle.id in self.scheduler.queue.running:
                assert((sumo_vehicle.index - 1) >= 0)
                segment_index = sumo_vehicle.index - 1
            else:
                segment_index = sumo_vehicle.index
            segment: SumoRoute = sumo_vehicle.segments[segment_index]
            edges: List[str] = route.attrib["edges"].split()
            current: int = traci.vehicle.getRouteIndex(sumo_vehicle.id)
            if segment.region_segment[0] != edges[0]:
                print(f"Error, segment[0] != edges[0] -> {segment.region_segment[0], edges[0]}")
                continue
            assert(segment.region_segment[-1] == edges[-1])
            assert(current < segment.first_edge_index)
            # TODO check if vehicle is on internal lane
            end: int = ((segment.first_edge_index-1) + len(segment.region_segment))
            new_route: List[str] = deepcopy(sumo_vehicle.route)
            assert(new_route[(segment.first_edge_index-1):end] == segment.region_segment)
            size_diff: int = len(edges) - len(segment.region_segment)
            new_route[(segment.first_edge_index-1):end] = edges
            try:
                traci.vehicle.setRoute(sumo_vehicle.id, new_route[current:])
                sumo_vehicle.route = new_route
                segment.planned_segment = edges
                for sumo_segment in sumo_vehicle.segments[(segment_index + 1):]:
                    # Change entry to network, if the segments are connected
                    if sumo_segment.region_segment[0] == segment.region_segment[-2]:
                        sumo_segment.region_segment[0] = edges[-2]
                    sumo_segment.first_edge_index += size_diff
                    end = ((sumo_segment.first_edge_index - 1) + len(sumo_segment.region_segment))
                    assert(sumo_vehicle.route[(sumo_segment.first_edge_index - 1):end] == sumo_segment.region_segment)
            except traci.exceptions.TraCIException as e:
                print(f"Error when assigning route to vehicle: {sumo_vehicle.id}")
                continue
        return True

    def get_steps(self, total_time: float, step_length: float) -> int:
        counter: int = 0
        while total_time > 0:
            total_time -= step_length
            counter += 1
        return max(counter, 1)


class PddlMain:
    def __init__(self, options: RoutingOptions):
        assert(options is not None)
        self.options: RoutingOptions = options
        self.mode: Optional[Mode] = None
        self.episodes_info: PddlInfo = PddlInfo()
        self._initialized: bool = False

    def run(self) -> bool:
        self.mode = DUO(self.options)
        # Run the planning
        self.mode.main()
        # Save the newly created routes along with vehicles
        if not self.mode.new_scenario.save(self.mode.scenario.config_file.get_network(), False):
            return False
        # Finally save config to given scenario
        config_path: str = self.mode.new_scenario.scenario_dir.info.format_file("config.json")
        return JsonFile(config_path).save(config_path, asdict(self.options))


if __name__ == '__main__':
    config: dict = JsonFile.load_config("duo_online")
    if not config or config is None:
        raise ValueError("Received invalid config!")
    pddl_main: PddlMain = PddlMain(RoutingOptions(config))
    pddl_main.run()
    # duo: DUO = DUO(Scenario("itsc_25200_32400"), "itsc_25200_32400", 180, 10)
    # duo.main()




