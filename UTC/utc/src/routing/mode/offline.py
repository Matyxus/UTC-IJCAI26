# from utc.src.routing.planning.mode import (
#     Mode, RoutingOptions, Scenario, Graph, RoadNetwork,
#     ProblemGenerator, ResultGenerator, Parser
# )
# from utc.src.routing.pddl.pddl_episode import PddlEpisode, PddlProblem, PddlResult
# from utc.src.utils.vehicle_extractor import VehicleExtractor, VehicleEntry
# import time
# from typing import Optional, List, Tuple, Iterator
#
#
# class Offline(Mode):
#     """ Class representing 'offline' mode of planning """
#     def __init__(self, options: RoutingOptions):
#         super().__init__(options)
#         self.vehicle_extractor: VehicleExtractor = VehicleExtractor(
#             self.scenario.vehicles_file, self.scenario.routes_file
#         )
#
#     def initialize(self) -> bool:
#         """
#         :return: True on success, false otherwise.
#         """
#         if self.options is None:
#             print("Received invalid PDDL options!")
#             return False
#         # Initialize scenarios
#         self.scenario = Scenario(self.options.init.scenario)
#         self.new_scenario = Scenario(self.options.init.new_scenario, True)
#         if not self.new_scenario.scenario_dir.initialize_dir(pddl=True, info_dir=True):
#             return False
#         elif not self.scenario.exists():
#             return False
#         # Initialize graph
#         self.graph: Graph = Graph(RoadNetwork())
#         if not self.graph.loader.load_map(self.scenario.config_file.get_network()):
#             return False
#         # Initialize sub-graph, if there is any
#         sub_graph: Optional[Graph] = None
#         if self.options.builder.regions[0] != "default":
#             sub_graph = Graph(RoadNetwork())
#             if not sub_graph.loader.load_map(self.options.builder.regions[0]):
#                 return False
#         # Initialize pddl classes
#         self.problem_generator = ProblemGenerator(self.new_scenario, self.options.builder, self.graph, sub_graph)
#         self.result_generator = ResultGenerator(self.new_scenario.scenario_dir)
#         self.parser = Parser(self.problem_generator.network_builder.graph, self.problem_generator.network_builder.sub_graph)
#         return True
#
#     def generate_episodes(self) -> Optional[List[PddlEpisode]]:
#         episodes: List[PddlEpisode] = []
#         # First generate all pddl problems
#         now: float = time.time()
#         it: Optional[Iterator[PddlProblem]] = self.generate_problems()
#         if it is None:
#             return None
#         problems: List[PddlProblem] = [problem for problem in it]
#         self.problem_generator = None  # Free memory of problem generator
#         print(f"Generated: {len(problems)} problems in: {round(time.time() - now, 3)} sec.")
#         # From generate problem files, generate results
#         results: List[Optional[PddlResult]] = self.result_generator.generate_results(
#             problems, self.options.init.mode.domain,
#             self.options.solver.name,
#             self.new_scenario.scenario_dir,
#             self.options.solver.timeout,
#             self.options.general.cpu.processes
#         )
#         if not results:
#             print("Error while generating pddl results!")
#             return None
#         assert(len(problems) == len(results))
#         # Generate pddl episodes classes
#         print("Generating episodes and saving results")
#         for i, (problem, result) in enumerate(zip(problems, results)):
#             episodes.append(PddlEpisode(i, problem, result))
#             assert(self.save_result(episodes[-1], free_mem=True))
#         return episodes
#
#     def generate_problems(self) -> Optional[Iterator[PddlProblem]]:
#         """
#         :return: generator of PddlProblem classes, None if error curred
#         """
#         epi_count, start_time = self.compute_time()
#         if epi_count <= 0 or start_time < 0:
#             print("Error, starting time and ending time of simulation are invalid!")
#             return None
#         window: int = self.options.init.mode.window
#         for i in range(1, epi_count+1):
#             print(f"***" * 15)
#             print(f"Generating pddl problem: {i}/{epi_count}")
#             # Get vehicles from vehicle file
#             entry: VehicleEntry = self.vehicle_extractor.estimate_arrival_naive(
#                 (start_time, start_time + self.options.init.mode.window)
#             )
#             if entry is None or not entry.vehicles:
#                 print(f"Unable to extract vehicles in interval: {start_time, start_time + window}")
#             else:
#                 problem: Optional[PddlProblem] = self.problem_generator.generate_problem(
#                     entry, f"problem_{start_time}_{start_time + window}", self.options.init.mode.domain
#                 )
#                 if problem is not None:
#                     yield problem
#             start_time += window
#             print(f"Finished pddl problem: {i}/{epi_count}")
#
#     # ----------------------------------- Utils -----------------------------------
#
#     def compute_time(self) -> Tuple[int, int]:
#         """
#         :return: Total number of episodes, start time
#         """
#         simulation_length: float = (
#             min(self.scenario.vehicles_file.get_end_time(), self.scenario.config_file.get_end_time()) -
#             max(self.scenario.vehicles_file.get_start_time(), self.scenario.config_file.get_start_time())
#         )
#         # print(f"Simulation length: {simulation_length}")
#         # We must add one to episodes, since this works as rounding up
#         epi_count: int = round(simulation_length / self.options.init.mode.window) + 1
#         start_time: int = int(max(self.scenario.vehicles_file.get_start_time(), self.scenario.config_file.get_start_time()))
#         return epi_count, start_time
#
