from typing import List, Tuple, Dict, Optional, Set

from scipy.sparse.csgraph import shortest_path

from utc.src.simulator.scenario import Scenario
from utc.src.graph import Graph, RoadNetwork, Junction, Route
from utc.src.routing.pddl.base.pddl_problem import PddlProblem
from utc.src.routing.pddl.base.vehicle_container import VehicleContainer, VehicleEntry
from utc.src.routing.pddl.domains.network_domain import NetworkDomain
from utc.src.utils.vehicle_extractor import VehicleExtractor
from utc.src.constants.file_system.my_file import MyFile
from utc.src.constants.static.file_constants import FilePaths, FileExtension
import time
import re


def create_problem():
    scenario: Scenario = Scenario("lust_25200_32400")
    extractor: VehicleExtractor = VehicleExtractor(scenario.vehicles_file, scenario.routes_file)
    entry: VehicleEntry = extractor.estimate_arrival_naive((25200, 25210))
    graph: Graph = Graph(RoadNetwork())
    assert(graph.loader.load_map("lust_central"))
    problem: PddlProblem = PddlProblem("test", "utc", network=graph.road_network, vehicles=VehicleContainer(entry))
    domain: NetworkDomain = NetworkDomain()
    domain.process_graph(problem)
    with open("test.pddl", "w+") as file:
        file.write(str(problem))


# ------------------------------ Pddl Graph ------------------------------

class PddlRoute:
    """ """
    def __init__(self, id: str, from_junction: str, to_junction: str):
        self.id: str = id
        self.internal_id: int = int(id[1:])
        self.from_junction: str = from_junction
        self.to_junction: str = to_junction

    def as_predicate(self) -> str:
        return f"(connected {self.from_junction} {self.id} {self.to_junction})"


class PddlJunction:
    """ """
    def __init__(self, id: str):
        self.id: str = id
        self.internal_id: int = self.extract_id(id)
        self.incoming: List[PddlRoute] = []
        self.outgoing: List[PddlRoute] = []

    def extract_id(self, id: str) -> int:
        if "v" in id:
            return -1
        elif "s" in id:
            return int(id.split("s")[0][1:])
        return int(id[1:])


class PddlNetwork:
    """ """
    def __init__(self):
        self.junctions: Dict[str, PddlJunction] = {}
        self.routes: Dict[str, PddlRoute] = {}
        self.vehicles: List[str] = []

    def load_network(self, problem_file: str) -> bool:
        if not MyFile.file_exists(problem_file):
            return False
        tmp = self.scan_tokens(problem_file)
        # ------------ Junctions ------------
        objects_part: List[str] = tmp[3]
        assert(objects_part[0] == ":objects")
        parts: List[str] = " ".join(objects_part[1:]).split("-")
        assert(parts[3].split()[0] == "use")
        self.vehicles = parts[3].split()[1:]
        assert(self.vehicles[-1].startswith("v"))
        for junction_id in parts.pop(0).split(" "):
            # Skip artificial junctions
            if not junction_id or "s" in junction_id or "v" in junction_id:
                continue
            self.junctions[junction_id] = PddlJunction(junction_id)
        # ------------ Connections ------------
        init_part: List[str] = tmp[4]
        assert(init_part[0] == ":init")
        for predicate in init_part[1:]:
            assert(isinstance(predicate, list))
            if predicate[0] != "connected":
                continue
            from_junction, route_id, to_junction = predicate[1], predicate[2], predicate[3]
            # Skip artificial vehicle starting/ending junctions
            if "v" in from_junction or "s" in from_junction or "v" in to_junction or "s" in to_junction:
                continue
            elif from_junction not in self.junctions or to_junction not in self.junctions:
                print(f"Invalid junction: {from_junction}, {to_junction}")
                return False
            # from_junction = (from_junction if "s" not in from_junction else from_junction.split("s")[0])
            # to_junction = (to_junction if "s" not in to_junction else to_junction.split("s")[0])
            self.routes[route_id] = PddlRoute(route_id, from_junction, to_junction)
            self.junctions[from_junction].outgoing.append(self.routes[route_id])
            self.junctions[to_junction].incoming.append(self.routes[route_id])
        return True

    def compare_networks(self, other: RoadNetwork) -> bool:
        """
        :param other:
        :return:
        """
        all_junctions: Set[int] = set([junction.internal_id for junction in self.junctions.values()])
        if -1 in all_junctions:
            all_junctions.remove(-1)
        all_routes: Set[int] = set([route.internal_id for route in self.routes.values()])
        if len(all_junctions)!= len(other.junctions):
            print(f"Incorrect number of junctions!")
            return False
        elif len(all_routes) != len(other.routes):
            print(f"Incorrect number of routes!")
            return False
        # Check all junctions for connections
        for pddl_junction in self.junctions.values():
            if pddl_junction.internal_id == -1:
                continue
            junction: Junction = other.get_junction(pddl_junction.internal_id)
            if junction is None:
                return False
            # ----- Check connections ----
            # Incoming routes
            for in_pddl_route in pddl_junction.incoming:
                route: Route = other.get_route(in_pddl_route.internal_id)
                if route is None or route not in junction.connections:
                    print(f"Invalid in-route: {in_pddl_route.id}({in_pddl_route.internal_id}) "
                          f"in junction {junction.id}(junction {junction.internal_id})")
                    return False
            # Outgoing routes
            out_routes: Set[int] = set([out_route.internal_id for out_route in junction.get_out_routes()])
            for out_pddl_route in pddl_junction.outgoing:
                route: Route = other.get_route(out_pddl_route.internal_id)
                if route is None or route.internal_id not in out_routes:
                    print(f"Invalid out-route: {out_pddl_route.id}({out_pddl_route.internal_id}) "
                          f"in junction {junction.id}(junction {junction.internal_id})")
                    return False
            # Connections
            if pddl_junction.incoming and pddl_junction.outgoing:
                for in_pddl_route in pddl_junction.incoming:
                    route: Route = other.get_route(in_pddl_route.internal_id)
                    out: Set[int] = set([route.internal_id for route in junction.travel(route)])
                    out_pddl: Set[int] = set([out_pddl_route.internal_id for out_pddl_route in pddl_junction.outgoing])
                    if not out:
                        print(f"Invalid connections in junction: {junction.id}({junction.internal_id})!")
                        print(f"PDDL representation has outgoing-routes, while network does not!")
                        print(f"{[in_route.as_predicate() for in_route in pddl_junction.incoming]} -> "
                              f"{[out_route.as_predicate() for out_route in pddl_junction.outgoing]}")
                        print(junction.info(True))
                        return False
                    elif out != out_pddl:
                        print(f"Invalid connections in junction: {junction.id}({junction.internal_id})!")
                        print(f"In: {in_pddl_route.id}({in_pddl_route.internal_id}), out: {out} != {out_pddl}")
                        return False
        print("Networks are equal!")
        return True

    def check_result_network(self, network: RoadNetwork, result_file: str) -> bool:
        """
        :param network:
        :param result_file:
        :return:
        """
        vehicles: Dict[str, List[str]] = self.parse_result(result_file)
        if vehicles is None or not vehicles:
            return False
        for vehicle, routes in vehicles.items():
            edges = network.get_edges([int(route_id[1:]) for route_id in routes])
            if None in edges or not network.check_edge_sequence(edges):
                print(f"Error at vehicle: {vehicle}, routes: {routes}")
                return False
        return True

    # ----------------------------- Utils -----------------------------

    def check_result_pddl(self, problem_file: str, result_file: str) -> bool:
        """
        :param result_file:
        :return:
        """
        if not (MyFile.file_exists(problem_file) and MyFile.file_exists(result_file)):
            return False
        tmp = self.scan_tokens(problem_file)
        init_part: List[str] = tmp[4]
        assert(init_part[0] == ":init")
        connections: Set[str] = set()
        for predicate in init_part[1:]:
            assert(isinstance(predicate, list))
            if predicate[0] != "connected":
                continue
            connections.add("(" + " ".join(predicate) + ")")
        vehicles: Dict[str, List[str]] = {}
        with open(result_file) as f:
            for line in f:
                line = line.strip()
                assert(line.startswith("(") and line.endswith(")"))
                line = line[1:-1].split()
                # (v0 jsv0 r483 j182s1 jev0)
                assert(len(line) == 5)
                if line[0] not in vehicles:
                    vehicles[line[0]] = []
                vehicles[line[0]].append("(connected " + " ".join(line[1:-1]) + ")")
        for vehicle, routes in vehicles.items():
            for connection in routes:
                if connection not in connections:
                    print(f"Error at vehicle: {vehicle}, connection: {connection} does not exist!")
                    return False
        return True

    def scan_tokens(self, filename):
        with open(filename) as f:
            # Remove single line comments
            context = re.sub(r';.*', '', f.read(), flags=re.MULTILINE).lower()
        # Tokenize
        stack = []
        tokens = []
        for t in re.findall(r'[()]|[^\s()]+', context):
            if t == '(':
                stack.append(tokens)
                tokens = []
            elif t == ')':
                if stack:
                    li = tokens
                    tokens = stack.pop()
                    tokens.append(li)
                else:
                    raise Exception('Missing open parentheses')
            else:
                tokens.append(t)
        if stack:
            raise Exception('Missing close parentheses')
        elif len(tokens) != 1:
            raise Exception('Malformed expression')
        return tokens[0]

    def parse_result(self, result_file: str) -> Optional[Dict[str, List[str]]]:
        """
        :param result_file:
        :return:
        """
        if not MyFile.file_exists(result_file):
            return None
        vehicles: Dict[str, List[str]] = {}
        with open(result_file) as f:
            for line in f:
                line = line.strip()
                assert(line.startswith("(") and line.endswith(")"))
                line = line[1:-1].split()
                # (v0 jsv0 r483 j182s1 jev0)
                assert(len(line) == 5)
                if line[0] not in vehicles:
                    vehicles[line[0]] = []
                vehicles[line[0]].append(line[2])
        return vehicles



if __name__ == '__main__':
    # create_problem()
    # problem_file: str = FilePaths.PDDL_PROBLEM.format("itsc_25200_32400_routed_mip", "problem_32360_32370_lust_central")
    # result_file: str = FilePaths.PDDL_RESULT.format("itsc_25200_32400_routed_mip", "result_32360_32370_lust_central")
    # assert(MyFile.file_exists(problem_file) and MyFile.file_exists(result_file))
    graph: Graph = Graph(RoadNetwork())
    graph.loader.load_map("DCC")
    print(graph.road_network.junctions["666094743"].info(True))
    # network: PddlNetwork = PddlNetwork()
    # network.load_network(FilePaths.PDDL_PROBLEM.format("DCC_central"))
    # print(network.compare_networks(graph.road_network))
    # network.check_result_pddl("test.pddl", result_file)
