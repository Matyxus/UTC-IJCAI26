from xml.etree.ElementTree import Element
from utc.src.constants.file_system.file_types.sumo_routes_file import SumoRoutesFile
from utc.src.constants.file_system.file_types.sumo_vehicles_file import SumoVehiclesFile
from utc.src.simulator.scenario import Scenario
from utc.src.constants.static.pddl_constants import SOLVERS
from utc.src.constants.static import DirPaths, FileExtension, FilePaths
from utc.src.constants.file_system.my_directory import MyDirectory, MyFile
from utc.src.constants.file_system.file_types.xml_file import XmlFile
from utc.src.constants.file_system.file_types.sumo_config_file import SumoConfigFile
from utc.src.graph import Graph, RoadNetwork
from utc.src.utils.task_manager import TaskManager
from ci_script import PddlNetwork
from vehicle_extractor import VehicleEntry, VehicleExtractor
from copy import deepcopy
from typing import Tuple, List, Set, Dict


class OldScenario:
    """
    Class representing the old scenario folder structure.
    """
    def __init__(self, name: str):
        self.name: str = name
        assert(MyDirectory.dir_exist(DirPaths.SCENARIO.format(name)))
        self.dir: MyDirectory = MyDirectory(DirPaths.SCENARIO.format(name))
        self.networks: Dict[str, str] = {
            MyFile.get_file_name(file_path): file_path for file_path in
            self.dir.get_sub_dir("maps").get_sub_dir("networks").list_dir(full_path=True, only_files=True)
            if file_path.endswith(FileExtension.SUMO_NETWORK)
        }
        assert(MyDirectory.dir_exist(DirPaths.PDDL_PROBLEMS.format(name)))
        assert(MyDirectory.dir_exist(DirPaths.SCENARIO.format(name) + "/results_milp"))
        self.problems: Dict[str, MyDirectory] = {
            MyFile.get_file_name(dir_path): MyDirectory(dir_path) for dir_path in
            self.dir.get_sub_dir("problems").list_dir(full_path=True, only_dirs=True)
        }
        self.results: Dict[str, MyDirectory] = {
            MyFile.get_file_name(dir_path): MyDirectory(dir_path) for dir_path in
            self.dir.get_sub_dir("results_milp").list_dir(full_path=True, only_dirs=True)
        }
        assert(len(self.results.keys()) == len(self.problems.keys()))
        self.routes: XmlFile = XmlFile(self.dir.get_sub_dir("routes").get_file(self.name + FileExtension.SUMO_ROUTES))
        assert(self.routes.is_loaded())

    def get_scenarios(self) -> List[Tuple[MyDirectory, MyDirectory, RoadNetwork]]:
        """
        :return: List of triples (problem directory, result directory, road network) with same name
        """
        triplets: List[Tuple[MyDirectory, MyDirectory, RoadNetwork]] = []
        for result_name, result_dir in self.results.items():
            graph: Graph = Graph(RoadNetwork())
            assert(graph.loader.load_map(self.networks[result_name.replace("_planned", "_sg")]))
            print(f"Loaded: {result_name.replace("_planned", "_sg")}, junctions: {len(graph.road_network.junctions)}")
            triplets.append((self.problems[result_name], result_dir, graph.road_network))
        return triplets



def parse_old(scenario_name: str) -> bool:
    print(f"Parsing {scenario_name}")
    scenario: OldScenario = OldScenario(scenario_name)
    sub_scenarios: List[Tuple[MyDirectory, MyDirectory, RoadNetwork]] = scenario.get_scenarios()
    extractor: VehicleExtractor = VehicleExtractor(
        SumoVehiclesFile(scenario.routes.file_path), SumoRoutesFile(scenario.routes.file_path)
    )
    entry: VehicleEntry = extractor.estimate_arrival_naive((0, float("inf")))
    tmp: Graph = Graph(sub_scenarios[0][2])
    # tmp.simplify.simplify_graph()
    print(len(sub_scenarios[0][2].junctions), sub_scenarios[0][0].name, sub_scenarios[0][1].name)
    return True


def parse_new(scenario_name: str) -> bool:
    # ---------- Scenario ----------
    original: str = "lust_25200_32400"
    scenario: Scenario = Scenario(original)
    assert(scenario.exists())
    network: Graph = Graph(RoadNetwork())
    assert(network.loader.load_map("lust"))
    # ---------- Graph ----------
    graph: Graph = Graph(RoadNetwork())
    assert(graph.loader.load_map("lust_red"))
    # ---------- Planned scenario ----------
    planned: str = "lust_25200_32400_planned_old_br"
    new_scenario: Scenario = Scenario(planned + "_mip", create_new=True)
    assert(MyDirectory.dir_exist(DirPaths.PDDL_PROBLEMS.format(planned)))
    extractor: VehicleExtractor = VehicleExtractor(scenario.vehicles_file, scenario.routes_file)
    changes: int = 0
    new_routes: Dict[str, str] = {}
    for i, problem_file in enumerate(MyDirectory.list_directory(DirPaths.PDDL_PROBLEMS.format(planned), full_path=True)):
        print(f"Parsing {problem_file}")
        # Get original vehicles in this interval
        timer = MyFile.get_file_name(problem_file).split("_")[1:]
        result_file: str = DirPaths.SCENARIO.format(planned) + f"/results_milp/result_{timer[0]}_{timer[1]}.pddl"
        # result_file: str = FilePaths.PDDL_RESULT.format(planned, f"result_{timer[0]}_{timer[1]}")
        if not MyFile.file_exists(result_file, message=False):
            continue
        entry: VehicleEntry = extractor.estimate_arrival_naive((int(timer[0]), int(timer[1])))
        id_mapping: Dict[str, str] = {f"v{i}": v_id for i, v_id in enumerate(entry.vehicles)}
        # Parse planned routes
        pddl_network: PddlNetwork = PddlNetwork()
        pddl_network.load_network(problem_file)
        assert(all([graph.road_network.route_exists(route_id, message=False) for route_id in pddl_network.routes]))
        assert(len(entry.vehicles) >= len(pddl_network.vehicles))
        # Substitute the original segments for the planned ones
        result = pddl_network.parse_result(result_file)
        for vehicle_id, pddl_routes in result.items():
            assert(vehicle_id in id_mapping)
            routes = graph.road_network.get_routes(pddl_routes)
            assert(None not in routes)
            edges = [edge for route in routes for edge in route.edge_list]
            assert(graph.road_network.check_edge_sequence(edges))
            vehicle = entry.vehicles[id_mapping[vehicle_id]]
            route = entry.original_routes[vehicle.attributes["route"]]
            segment = graph.road_network.get_longest_sequence(route.attrib["edges"].split())[0]
            orig, new = " ".join([edge.id for edge in segment]), " ".join([edge.id for edge in edges])
            # No change
            if orig == new:
                continue
            changes += 1
            # print(f"Replacing: {orig} by: {new}")
            new_routes[id_mapping[vehicle_id]] = route.attrib["edges"].replace(orig, new)
            assert(route.attrib["edges"].split()[0] == new_routes[id_mapping[vehicle_id]].split()[0])
            assert(route.attrib["edges"].split()[-1] == new_routes[id_mapping[vehicle_id]].split()[-1])
            # Check against whole network for validity
            assert(network.road_network.check_edge_sequence(new_routes[id_mapping[vehicle_id]].split()))
    # Save new routes for vehicles into new scenario
    for vehicle, route in extractor.estimate_arrival_naive((0, float("inf"))):
        vehicle = vehicle.to_xml()
        if vehicle.attrib["id"] in new_routes:
            vehicle.attrib["route"] = new_scenario.routes_file.add_route(
                Element("route", {"id": route.attrib["id"], "edges": new_routes[vehicle.attrib["id"]]}),
                re_index=True
            )
        else:
            vehicle.attrib["route"] = new_scenario.routes_file.add_route(route, re_index=True)
        new_scenario.vehicles_file.add_vehicle(vehicle)
    print(f"Changed: {changes} vehicle routes")
    return new_scenario.save(FilePaths.MAP_SUMO.format("lust"), with_directory=True)

if __name__ == '__main__':
    parse_new("")




