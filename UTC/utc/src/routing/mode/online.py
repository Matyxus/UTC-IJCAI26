from utc.src.routing.base.traffic_problem import TrafficProblem
from utc.src.routing.mode.mode import Mode, RoutingOptions
from utc.src.routing.control.scheduler import Scheduler, ControlledVehicle, Segment
from utc.src.routing.traffic.dso import DSO
from utc.src.routing.traffic.duo import DUO
from utc.src.graph import Route
from utc.src.simulator.simulation import Simulation, traci
# from xml.etree.ElementTree import Element
# from copy import deepcopy
from typing import Optional, List, Set, Tuple, Dict
import time

class Online(Mode):
    """ Class representing 'online' mode of vehicle routing """

    def __init__(self, options: RoutingOptions):
        self.dso: Optional[DSO] = None
        self.duo: Optional[DUO] = None
        super().__init__(options)
        self.scheduler: Scheduler = Scheduler(self.graph.road_network, self.options.init.mode.reserve)

    def initialize(self) -> bool:
        if not super().initialize():
            return False
        # Assign starting travel time to edges (free-flow travel time)
        for edge in self.graph.road_network.get_edge_list():
            edge.attributes["travelTime"] = edge.get_travel_time()
            edge.attributes["region"] = -1
        # Copy travel times to regions to be updated automatically
        for region_id, region in enumerate(self.sub_graphs):
            for edge in region.road_network.get_edge_list():
                edge.attributes = self.graph.road_network.get_edge(edge.id).attributes
                edge.attributes["region"] = region_id
        # TODO Initialize DSO/DUO if enabled
        self.dso = DSO(self.new_scenario, self.sub_graphs, self.options.builder)
        self.duo = DUO(self.graph, self.sub_graphs)
        # At least one routing type has to be active
        assert(self.duo is not None or self.dso is not None)
        return True

    # -------------------------------------------- Simulation --------------------------------------------

    def run(self) -> List[ControlledVehicle]:
        step_duration: float = self.scenario.config_file.get_step_length() # Duration of single simulation step (sec)
        traffic_steps: int = int(self.options.init.mode.window / step_duration) # Number of steps in single traffic window
        step, total_steps = 0, int((self.scenario.config_file.get_duration() / step_duration) / traffic_steps)
        scheduled: List[ControlledVehicle] = []
        dso_routes: List[Optional[Route]] = []
        dso_time: float = 0.
        with Simulation(self.scenario.scenario_dir.get_config(self.options.init.config), {"-W": ""}) as simulation:
            while simulation.is_running():
                print(f"-------- Time: {simulation.get_time(False)}, step: {step+1}/{total_steps} --------")
                # ---------------------- Advance ----------------------
                # Advance simulation by given traffic window
                for planning_step in range(1, traffic_steps + 1):
                    simulation.step()
                    departed_vehicles: List[ControlledVehicle] = self.scheduler.step(simulation)
                    # Check if simulation is still running
                    if not simulation.is_running():
                        break
                    # Check for DUO - immediate vehicle routing for each new entry (if enabled)
                    if self.duo is not None and departed_vehicles:
                        duo_routes, duo_time = self.duo.route_vehicles(departed_vehicles)
                        # print(f"Duo found: {(len(duo_routes) - duo_routes.count(None))}/{len(departed_vehicles)} routes in {duo_time}[s].")
                        self.scheduler.assign_routes(departed_vehicles, duo_routes, "DUO")
                        # TODO Check route from DUO for segment changes
                    # DSO delayed assigment (previous traffic window - to account for time needed)
                    if (planning_step * step_duration) >= dso_time and dso_routes:
                        self.scheduler.assign_routes(scheduled, dso_routes, "DSO")
                        dso_routes = []
                dso_routes = [] # Make sure previous assigment is reset after traffic window passed
                # Check if simulation is still running
                if not simulation.is_running():
                    break
                # ---------------------- Schedule ----------------------
                self.scheduler.update_travel_time()
                if self.sub_graphs: # There is only global DUO available without controlled regions
                    scheduled: List[ControlledVehicle] = self.scheduler.schedule_vehicles(self.options.init.mode.interval[1])
                else:
                    scheduled: List[ControlledVehicle] = []
                # Generate problems & results, pretend this is asynchronous, so results will be ready next step
                if scheduled:
                    # DUO can be assigned immediately, as it is very fast
                    if self.duo is not None:
                        duo_routes, duo_time = self.duo.route_vehicles(scheduled)
                        self.scheduler.assign_routes(scheduled, duo_routes, "DUO")
                    # Generate DSO routes asynchronously (i.e. assign after the time needed already passed)
                    if self.dso is not None:
                        dso_routes, dso_time = self.dso.route_vehicles(scheduled)
                        # print(f"DSO generated result in: {dso_time}[s].")
                # Continue to next step (traffic window)
                step += 1
        return list(self.scheduler.queue.vehicles.values())

    # -------------------------------------------- Episodes --------------------------------------------

    # def generate_episode(self, scheduled: List[SumoVehicle], simulation: Simulation) -> Optional[List[PddlEpisode]]:
    #     """
    #     :param scheduled: Vehicles scheduled for planning
    #     :param simulation: Current running SUMO simulation
    #     :return: List of generated PddlEpisodes (each region has its own problem & result pair)
    #     """
    #     assert (simulation is not None and simulation.is_running(use_end_time=False))
    #     if scheduled is None or not scheduled:
    #         return None
    #     # Generate PddlProblems for scheduled vehicles
    #     problems: List[PddlProblem] = self.generate_problems(scheduled, simulation)
    #     if problems is None or not problems:
    #         return None
    #     # Run planner for generated problems (at most len(regions) at once)
    #     assert (len(problems) <= 3)
    #     results: List[Optional[PddlResult]] = self.result_generator.generate_results(
    #         problems, self.options.init.mode.domain,
    #         self.options.solver.name, self.new_scenario.scenario_dir,
    #         self.options.solver.timeout, len(problems)
    #     )
    #     if results is None or not results:
    #         print("Error while generating pddl results!")
    #         return None
    #     return [PddlEpisode(self.episode_counter + i, problem, result) for i, (problem, result) in enumerate(zip(problems, results))]
    #
    # def generate_problems(self, scheduled: List[SumoVehicle], simulation: Simulation) -> List[Optional[PddlProblem]]:
    #     """
    #     :return:
    #     """
    #     assert(simulation is not None and simulation.is_running(use_end_time=False))
    #     interval: Tuple[int, int] = (int(simulation.get_time() - self.options.init.mode.window), int(simulation.get_time()))
    #     if not scheduled:
    #         print(f"No vehicles scheduled in interval: {interval}")
    #         return []
    #     # Split vehicles based on their regions, create PddlProblem for each
    #     entries: List[VehicleEntry] = [VehicleEntry(interval) for _ in range(len(self.sub_graphs))]
    #     for sumo_vehicle in scheduled:
    #         segment: SumoRoute = sumo_vehicle.get_current_segment()
    #         route_id: str = sumo_vehicle.get_attribute("route")
    #         region_id: int = segment.region_id
    #         entries[region_id].vehicles[sumo_vehicle.id] = deepcopy(sumo_vehicle)
    #         entries[region_id].original_routes[route_id] = Element("route", {"id": route_id, "edges": " ".join(segment.region_segment)})
    #     problems: List[Optional[PddlProblem]] = []
    #     # Generate problem for each non-empty vehicle entry
    #     for region_id, entry in enumerate(entries):
    #         if not entry.vehicles:
    #             continue
    #         problems.append(
    #             self.problem_generator[region_id].generate_problem(
    #                 entry, f"problem_{interval[0]}_{interval[1]}_{self.options.builder.regions[region_id]}",
    #                 self.options.init.mode.domain
    #             )
    #         )
    #     return [problem for problem in problems if problem is not None and problem.is_valid()]
    #
    # # ------------------------------- Utils -------------------------------
    #
    # def parse_results(self, episodes: List[TrafficProblem]) -> List[Dict[str, List[str]]]:
    #     """
    #     :param episodes:
    #     :return:
    #     """
    #     if not episodes:
    #         return []
    #     ret_val: List[Dict[Element, Element]] = []
    #     for episode in episodes:
    #         region_id: int = -1
    #         for i, region_name in enumerate(self.options.builder.regions):
    #             if episode.problem.name.endswith(region_name):
    #                 region_id = i
    #                 break
    #         assert(region_id != -1)
    #         parsed: Optional[Dict[Element, Element]] = self.parser[region_id].process_result(episode)
    #         if parsed is None or not parsed:
    #             print(f"Unable to parse problem: {episode.problem.name}")
    #             continue
    #         ret_val.append(parsed)
    #         episode.free_mem()
    #     return ret_val



