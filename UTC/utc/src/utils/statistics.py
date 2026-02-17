from utc.src.constants.file_system.my_file import MyFile
from utc.src.simulator.scenario import Scenario
from utc.src.constants.static import DirPaths, FileExtension, FilePaths
from utc.src.constants.static.pddl_constants import SOLVERS
from utc.src.constants.file_system.my_directory import MyDirectory
from utc.src.constants.file_system.file_types.xml_file import XmlFile
from utc.src.constants.file_system.file_types.sumo_config_file import SumoConfigFile
from utc.src.graph import Graph, RoadNetwork
from utc.src.utils.task_manager import TaskManager
from copy import deepcopy
from typing import Tuple, List, Union
import csv


# def merge_scenarios(scenarios: List[str], scenario_name: str, sort_vehicles: bool = False) -> bool:
#     """
#     :param scenarios: list of scenarios name to be merged
#     :param scenario_name:
#     :param sort_vehicles
#     :return:
#     """
#     print(f"Merging scenarios: {scenarios}")
#     if len(scenarios) < 2:
#         print("Expected at least 2 scenarios to be merged!")
#         return False
#     elif MyDirectory.dir_exist(DirPaths.SCENARIO.format(scenario_name), message=False):
#         print(f"Cannot create new scenario: {scenario_name}, already exists!")
#         return False
#     new_scenario: Scenario = Scenario(scenario_name, create_new=True)
#     network_file: str = ""
#     for name in scenarios:
#         scenario: Scenario = Scenario(name)
#         if not scenario.exists(True):
#             return False
#         network_file = scenario.config_file.get_network()
#         mapping = new_scenario.routes_file.add_routes(scenario.routes_file.root.findall("route"), re_index=True)
#         for vehicle in scenario.vehicles_file.root.findall("vehicle"):
#             vehicle.attrib["route"] = mapping[vehicle.attrib["route"]]
#             new_scenario.vehicles_file.add_vehicle(vehicle)
#     if sort_vehicles:
#         new_scenario.vehicles_file.root[1:] = sorted(
#             new_scenario.vehicles_file.root[1:], key=lambda x: float(x.attrib["depart"])
#         )
#     return new_scenario.save(network_file)

def get_mapping(scenario: Scenario) -> dict:
    """
    :param scenario:
    :return:
    """
    routes: dict = {route.attrib["id"]: route for route in scenario.routes_file.root.findall("route")}
    return {
        vehicle.attrib["id"]: (vehicle, deepcopy(routes[vehicle.attrib["route"]]))
        for vehicle in scenario.vehicles_file.root.findall("vehicle")
    }


def find_change(original_edges: str, new_edges: str) -> Tuple[str, str]:
    """
    :param original_edges:
    :param new_edges:
    :return:
    """
    original_list: List[str] = original_edges.split()
    new_list: List[str] = new_edges.split()
    left_index: int = 0
    for i in range(min(len(original_list), len(new_list))):
        if original_list[i] != new_list[i]:
            left_index = i
            break
    right_index: int = 0
    for i in range(-1, -min(len(original_list), len(new_list)) + left_index, -1):
        if original_list[i] != new_list[i]:
            right_index = i+1
            break
    if right_index == 0:
        return " ".join(original_list[left_index:]), " ".join(new_list[left_index:])
    return " ".join(original_list[left_index:right_index]), " ".join(new_list[left_index:right_index])


def merge_planned_scenarios(original: str, scenario_name: str, planned_scenarios: List[str]) -> bool:
    """
    :param original:
    :param planned_scenarios:
    :return:
    """
    print(f"Merging planned scenarios: {planned_scenarios} into original: {original}")
    original_scenario: Scenario = Scenario(original)
    if not original_scenario.exists():
        return False
    scenarios: List[Scenario] = [Scenario(planned_name) for planned_name in planned_scenarios]
    assert(all([scenario.exists() for scenario in scenarios]))
    # Mapping of vehicle ids to their route
    original_mapping: dict = get_mapping(original_scenario)
    # Among all planned scenarios, find which vehicle changed its route
    changes: dict = {
        # Vehicle id : [(replace_seq, by_seq), ...], ....
    }
    for planned_scenario in scenarios:
        print(f"Processing planned scenario: {planned_scenario.name}")
        planned_mapping: dict = get_mapping(planned_scenario)
        assert(len(planned_mapping.keys() & original_mapping.keys()) == len(planned_mapping))
        for vehicle_id, (vehicle, route) in planned_mapping.items():
            # No changes
            if route.attrib["edges"] == original_mapping[vehicle_id][1].attrib["edges"]:
                continue
            # Route was changed
            original_edges: str = original_mapping[vehicle_id][1].attrib["edges"]
            replace_seq, replace_by = find_change(original_edges, route.attrib["edges"])
            assert(original_edges.replace(replace_seq, replace_by) == route.attrib["edges"])
            if vehicle_id not in changes:
                changes[vehicle_id] = []
            changes[vehicle_id].append((replace_seq, replace_by))
    # Generate new scenario
    new_scenario: Scenario = Scenario(scenario_name, create_new=True)
    for vehicle_id, (vehicle, route) in original_mapping.items():
        # Change route
        if vehicle_id in changes:
            for (replace_seq, replace_by) in changes[vehicle_id]:
                if not (replace_seq in route.attrib["edges"]):
                    print(f"Error at vehicle: {vehicle_id}, replacing edges: {set(replace_seq.split()) ^ set(replace_by.split())}")
                    continue
                route.attrib["edges"] = route.attrib["edges"].replace(replace_seq, replace_by)
            changes.pop(vehicle_id)
        # Save route to scenario
        route_id: str = new_scenario.routes_file.add_route(route)
        vehicle.attrib["route"] = route_id
        new_scenario.vehicles_file.add_vehicle(vehicle)
    return new_scenario.save(original_scenario.config_file.get_network())


def check_routes(original: str, planned: str) -> bool:
    """
    :param original:
    :param planned:
    :return:
    """
    print(f"Checking scenario: {original}, and planned: {planned}")
    original_scenario: Scenario = Scenario(original)
    planned_scenario: Scenario = Scenario(planned)
    if not original_scenario.exists():
        return False
    elif not planned_scenario.exists():
        return False
    original_mapping: dict = get_mapping(original_scenario)
    planned_mapping: dict = get_mapping(planned_scenario)
    print(len(original_mapping), len(planned_mapping))
    # assert(len(original_mapping.keys() & planned_mapping.keys()) == len(original_mapping.keys()))
    unchanged: int = 0
    changed: int = 0
    first_changed: int = 0
    last_changed: int = 0
    graph: Graph = Graph(RoadNetwork())
    assert(graph.loader.load_map(original_scenario.config_file.get_network()))
    for vehicle, route in planned_mapping.values():
        planned_edges: list = graph.road_network.get_edges(route.attrib["edges"].split())
        assert(None not in planned_edges)
        assert(graph.road_network.check_edge_sequence(planned_edges))
        original_edges: list = graph.road_network.get_edges(original_mapping[vehicle.attrib["id"]][1].attrib["edges"].split())
        assert(None not in original_edges)
        if planned_edges[0].from_junction != original_edges[0].from_junction:
            print(f"Error at route: {route.attrib['id']}, {original_mapping[vehicle.attrib['id']][1].attrib['id']}")
            print(f"Vehicle: {vehicle.attrib['id']}")
        assert(planned_edges[0].from_junction == original_edges[0].from_junction)
        assert(planned_edges[-1].to_junction == original_edges[-1].to_junction)
        first_changed += (planned_edges[0].id != original_edges[0].id)
        last_changed += (planned_edges[-1].id != original_edges[-1].id)
        if route.attrib["edges"] == original_mapping[vehicle.attrib["id"]][1].attrib["edges"]:
            unchanged += 1
        else:
            first, second = find_change(original_mapping[vehicle.attrib["id"]][1].attrib["edges"], route.attrib["edges"])
            assert(
                original_mapping[vehicle.attrib["id"]][1].attrib["edges"].replace(first, second) == route.attrib["edges"]
            )
            changed += 1
    print(f"Changed: {changed}/{len(planned_mapping)} vehicle's routes")
    print(f"Vehicles changed first/last edges: {(first_changed, last_changed)}")
    return True


class StatisticsGenerator:
    """
    Class generating scenario's statistical files.
    """
    def __init__(self, max_task: int = 4):
        """
        :param max_task:
        """
        self.max_tasks: int = max_task
        self.dump_command: str = "sumo -c {0} -W"
        self.statistics_command: str = "sumo -c {0} -W --duration-log.statistics true --statistic-output {1}.xml"
        self.trip_command: str = " --tripinfo-output {0}_tripinfo.xml --device.emissions.probability 1"
        self.template: List[str] = [
            'name',  # file name
            'loaded', 'inserted', 'running', 'waiting',  # vehicles
            'total', 'jam', 'yield lane', 'wrong lane',  # teleports
            'collisions', 'stops', 'braking',  # safety
            # VehicleTripsStatistics
            'vehicles', 'routeLength', 'speed', 'duration', 'waiting time',
            'timeLoss', 'departDelay', 'total travel time', 'total depart delay'
        ]

    # -------------------- Statistics --------------------

    def generate_dump(
            self, configs: List[Tuple[str, str]],
            period: float, statistics: bool = False
        ) -> bool:
        """
        :param configs: List of pairs (scenario_name, config_name)
        :param period: Frequency of edgeData aggregation (seconds)
        :param statistics: True if statistics should also be generated, False by default
        :return: True on success, false otherwise
        """
        print(f"Generating edgeData files for: {configs}")
        task_manager: TaskManager = TaskManager(min(len(configs), self.max_tasks))
        command: str = (self.dump_command if not statistics else self.statistics_command)
        for scenario_name, config_name in configs:
            config_file: SumoConfigFile = SumoConfigFile(FilePaths.SCENARIO_CONFIG.format(scenario_name, config_name))
            statistics_dir_path: str = DirPaths.SCENARIO_STATISTICS.format(scenario_name)
            # Check correctness
            if not config_file.is_loaded():
                return False
            elif not (MyDirectory.dir_exist(statistics_dir_path) or MyDirectory.make_directory(statistics_dir_path)):
                return False
            dump_file: XmlFile = XmlFile(FilePaths.XmlTemplates.EDGE_DATA)
            data = dump_file.root.find("edgeData")
            data.attrib["file"] = f"{config_name}_edgeData" + FileExtension.EDGE_DUMP
            data.attrib["period"] = str(period)
            dump_path: str = DirPaths.SCENARIO_CONFIGS.format(scenario_name) + f"/{config_name}_edgeData.add.xml"
            config_file.set_additional_file(dump_path)
            if not (dump_file.save(dump_path) and config_file.save()):
                return False
            # More format arguments are ignored, no need to differentiate
            task_manager.tasks.append((
                TaskManager.call_shell_block,
                tuple([command.format(config_file.file_path, config_name), statistics_dir_path])
            ))
        task_manager.start()
        return True


    def generate_statistics(self, configs: List[Tuple[str, str]], trip_info: bool = False) -> bool:
        """
        :param configs: List of pairs (scenario_name, config_name)
        :param trip_info: True if TripInfo (including Co2 emissions) should be generated
        :return: True on success, false otherwise
        """
        task_manager: TaskManager = TaskManager(min(len(configs), self.max_tasks))
        for scenario_name, config_name in configs:
            config_path: str = FilePaths.SCENARIO_CONFIG.format(scenario_name, config_name)
            statistics_dir_path: str = DirPaths.SCENARIO_STATISTICS.format(scenario_name)
            # Check correctness
            if not XmlFile.file_exists(config_path, message=True):
                return False
            elif not (MyDirectory.dir_exist(statistics_dir_path) or MyDirectory.make_directory(statistics_dir_path)):
                return False
            command: str = self.statistics_command.format(config_path, MyFile.get_file_name(config_name))
            if trip_info:
                command += self.trip_command.format(MyFile.get_file_name(config_name))
            task_manager.tasks.append((TaskManager.call_shell_block, tuple([command, statistics_dir_path])))
        task_manager.start()
        return True

    # -------------------- Utils --------------------

    def prepare_config(
            self, config_file: SumoConfigFile, interval: Tuple[float, float],
            additional_files: List[str] = None, step_length: float = 0.5,
            routing: bool = False
        ) -> bool:
        """
        :param config_file: config file
        :param interval: time interval of vehicle start and end
        :param additional_files: list of additional files to be included
        :param step_length: step length
        :param routing: True if routing should be added, false otherwise
        :return: True on success, false otherwise
        """
        if not config_file.is_loaded():
            return False
        config_file.set_begin(interval[0])
        config_file.set_end(interval[1])
        if additional_files is not None:
            for file in additional_files:
                config_file.set_additional_file(file)
        if step_length > 0:
            config_file.set_step_length(step_length)
        if routing:
            config_file.add_routing()
        return config_file.save()

    def format_statistics(self, scenario: Scenario, file_name: str = "") -> bool:
        """
        :param scenario: scenario of which statistics will be formatted into CSV file
        :param file_name: Name of file to be generated, by default 'results.csv'
        :return: True on success, false otherwise
        """
        if not scenario.exists(message=True) or not scenario.scenario_dir.stats.list_dir(True, True, True):
            print(f"Error, scenario: '{scenario.name}' is not valid!")
            return False
        stats: List[List[str]] = [self.template]
        for statistics_file in scenario.scenario_dir.stats.list_dir(True, True, True):
            if not statistics_file.endswith("xml"):
                continue
            xml_file: XmlFile = XmlFile(statistics_file, "r")
            assert(xml_file.is_loaded())
            veh_stats = xml_file.root.find("vehicleTripStatistics").attrib
            veh_stats.pop("departDelayWaiting")
            # Name, vehicles, teleports, safety, TripStatistics
            stats.append(
                [xml_file.get_name()] +
                list(xml_file.root.find("vehicles").attrib.values()) +
                list(xml_file.root.find("teleports").attrib.values()) +
                list(xml_file.root.find("safety").attrib.values()) +
                list(veh_stats.values())
            )
        # Format statistics into CSV file
        file_name = "results" if not file_name else file_name
        file_name = (file_name + ".csv") if not file_name.endswith(".csv") else file_name
        csv_file: str = scenario.scenario_dir.stats.format_file(file_name)
        try:
            with open(csv_file, 'w+', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(stats)
        except IOError as e:
            print(f"Error: {e} while creating CSV file: '{csv_file}'")
            return False
        return True

# if itsc:
#     scenario.config_file.set_additional_file(DirPaths.SCENARIO.format("Base") + "/DCC_trafficlights.add.xml")
# else:
#     scenario.config_file.set_additional_file(DirPaths.SCENARIO_ADDITIONAL.format("Lust") + "/tll.static.xml")


if __name__ == "__main__":
    statistics_generator: StatisticsGenerator = StatisticsGenerator()
    scenarios: List[str] = ["lust_25200_32400_comb_test", "itsc_25200_32400_comb_test2"]
    # for scenario_name in scenarios:
    #     statistics_generator.prepare_config(
    #         SumoConfigFile(FilePaths.SCENARIO_CONFIG.format(scenario_name, scenario_name)),
    #         (25200, 32400), #(18000, 27000) #,  # #,
    #         #[DirPaths.SCENARIO.format("Base") + "/DCC_trafficlights.add.xml"]
    #        [DirPaths.SCENARIO_ADDITIONAL.format("Lust") + "/tll.static.xml"]
    #     )
    # statistics_generator.generate_statistics([(scenario, scenario)])
    # statistics_generator.generate_dump(
    #     [# ("itsc_25200_32400_dso_mip_allowed", "itsc_25200_32400_dso_mip_allowed"),
    #      # ("lust_25200_32400_dso_mip_allowed", "lust_25200_32400_dso_mip_allowed"),
    #      ("itsc_25200_32400_planned3", "itsc_25200_32400_planned3")],
    #     period=900, statistics=True
    # )
    statistics_generator.generate_dump(
        [(scenario_name, scenario_name) for scenario_name in scenarios],
        period=900, statistics=True
    )
