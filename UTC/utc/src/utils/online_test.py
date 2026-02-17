import time
from utc.src.simulator.scenario import Scenario
from utc.src.simulator.simulation import Simulation
from utc.src.routing.pddl.base.pddl_problem import PddlProblem, VehicleContainer
from utc.src.constants.static import DirPaths, FileExtension, FilePaths
from utc.src.constants.file_system.my_file import MyFile
from utc.src.constants.file_system.file_types.sumo_vehicles_file import SumoVehiclesFile
from utc.src.constants.file_system.file_types.sumo_routes_file import SumoRoutesFile
from utc.src.routing.mode.scheduler import Scheduler, SumoVehicle, SumoRoute, VehicleStats
from utc.src.utils.vehicle_extractor import VehicleExtractor, VehicleEntry
from utc.src.routing.pddl.pddl_episode import PddlEpisode, PddlProblem, PddlResult
from utc.src.routing.mode.scheduler import Scheduler, SumoVehicle, SumoRoute, VehicleStats
from utc.src.routing.traffic.network_builder import NetworkBuilder
from utc.src.routing.pddl.domains.network_domain import NetworkDomain
from utc.src.routing.pddl.domains.vehicle_domain import VehicleDomain
from utc.src.constants.static.pddl_constants import SOLVERS
from utc.src.constants.file_system.my_directory import MyDirectory
from utc.src.constants.file_system.file_types.xml_file import XmlFile
from utc.src.constants.file_system.file_types.sumo_config_file import SumoConfigFile
from utc.src.graph import Graph, RoadNetwork
from utc.src.utils.task_manager import TaskManager
from copy import deepcopy
from typing import Tuple, List, Optional
import traci
from matplotlib import pyplot as plt
import xml.etree.ElementTree as ET

def is_file_available(file_path):
    try:
        # Try to open the file for reading or writing
        with open(file_path, 'r+') as file:
            content: str = file.read()
            print(content[-50:])
            # content += "</meandata>"
            # if not content.endswith("</meandata>"):
            #     content += "</meandata>"
            # print(content[-100:])
            # file.write(content)
            return True
    except IOError:
        return False

def generate_image(values: List[float], period: float = 300) -> bool:
    n_vehicles = len(values)
    print(f"Generating image of traffic intensity for: '{n_vehicles}' vehicles.")
    print(f"Start time: {values[0]}, end time: {values[-1]}")
    intervals: int = int((values[-1] - values[0]) // period)
    hours: int = int((values[-1] - values[0]) // 3600)
    print(f"Total number of hours in scenario: {hours}, intervals: {intervals}, period: {period}")
    # Data
    x: list = []
    y: list = []
    start = 0
    for i in range(intervals):
        limit: float = (i + 1) * period
        count: int = 0
        while values[start] < limit:
            start += 1
            count += 1
        y.append(count)
        x.append(i*period)
    average: float = sum(y) / len(y)  # average amount of vehicles per period
    deviation: float = (sum([(i - average) ** 2 for i in x]) / len(y)) ** (1/2)
    # Plot
    plt.xlim((0, max(x)))
    plt.ylim((0, max(y)+1000))
    plt.plot(x, y)
    print(f"Average: {average}, deviation: {deviation}")
    # plt.plot(x, [average] * len(x), label='Mean', linestyle='--', color="red")
    plt.xticks([i*3600 for i in range(hours+1)], labels=[i for i in range(hours+1)])
    plt.yticks([i*1000 for i in range((max(y) // 1000)+2)])
    plt.xlabel("Time [hours]")
    plt.ylabel("Vehicles [thousands]")
    plt.title("Traffic intensity")
    plt.tight_layout()
    plt.show()
    return True

if __name__ == "__main__":
    graph: Graph = Graph(RoadNetwork())
    assert(graph.loader.load_map("ingolstadt"))
    vehicle_file: SumoVehiclesFile = SumoVehiclesFile()
    routes_file: SumoRoutesFile = SumoRoutesFile()
    departs: List[float] = []
    n_vehicles: int = 0
    files: List[str] = ["InTAS_002", "InTAS_003", "InTAS_004", "InTAS_005", "InTAS_006"]
    for file_path in MyDirectory.list_directory("routes", full_path=True):
        file: XmlFile = XmlFile(file_path)
        print(f"Parsing {file.get_name()}")
        if file.get_name() not in files:
            continue
        for vehicle in file.root.findall("vehicle"):
            if not (18000 <= float(vehicle.attrib["depart"]) <= 27000):
                continue
            best_route = None
            for route in vehicle.find("routeDistribution").findall("route"):
                if len(route.attrib["edges"].split()) <= 1:
                    continue
                elif best_route is None or float(best_route.attrib["probability"]) < float(route.attrib["probability"]):
                    best_route = route
            if best_route is None:
                continue
            best_route.attrib["id"] = "unk"
            best_route.attrib.pop("probability")
            best_route.attrib.pop("cost")
            new_id: str = routes_file.add_route(best_route, True)
            vehicle.attrib["route"] = new_id
            vehicle.attrib["type"] = "CarDefault"
            vehicle.attrib["departLane"] = "best"
            vehicle.attrib["departPos"] = "random_free"
            vehicle.attrib["departSpeed"] = "max"
            vehicle.attrib["arrivalPos"] = "max"
            vehicle.remove(vehicle.find("routeDistribution"))
            vehicle.text = None
            vehicle.tail = None
            vehicle_file.add_vehicle(vehicle)
    scenario: str = "intas"
    vehicle_file.save(FilePaths.SCENARIO_VEHICLES.format(scenario, scenario))
    routes_file.save(FilePaths.SCENARIO_ROUTES.format(scenario, scenario))

    # generate_image(departs, period=300)

    #         route = vehicle.find("routeDistribution").find("route")
    #         edges = route.attrib["edges"].split()
    #         starting_edges[edges[0]] += 1
    #         ending_edges[edges[-1]] += 1
    # print(f"Number of vehicles: {n_vehicles}")
    # starting_edges = {k: v for k, v in sorted(starting_edges.items(), key=lambda item: item[1], reverse=True)}
    # ending_edges = {k: v for k, v in sorted(ending_edges.items(), key=lambda item: item[1], reverse=True)}
    # # print(sum([v for v in starting_edges.values()][:10]))
    # # print(sum([v for v in ending_edges.values()][:10]))
    # fig, ax = graph.display.initialize_plot()
    # graph.display.render_edges(ax, graph.road_network.get_edge_list())
    # graph.display.render_edges(ax, graph.road_network.get_edges([k for k in starting_edges.keys()][:10]), colors="green")
    # graph.display.render_edges(ax, graph.road_network.get_edges([k for k in ending_edges.keys()][:10]), colors="red")
    # graph.display.show_plot(ax)




    # with Simulation(FilePaths.SCENARIO_CONFIG.format("lust_25200_32400", "edgeData_test"), {"-W": ""}) as simulation:
    #     while simulation.is_running() and traci.simulation.getTime() <= 25500:
    #         simulation.step()
    #         if traci.simulation.getTime() % 10 == 0:
    #             print(f"Time: {traci.simulation.getTime()}")
    #         if traci.simulation.getTime() >= 25400:
    #             # print(MyFile.file_exists(DirPaths.SCENARIO_CONFIGS.format("lust_25200_32400") + "/edgeData.out.xml"))
    #             is_file_available(DirPaths.SCENARIO_CONFIGS.format("lust_25200_32400") + "/edgeData2.out.xml")




