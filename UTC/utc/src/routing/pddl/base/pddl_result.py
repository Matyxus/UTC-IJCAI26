from utc.src.constants.static.file_constants import FileExtension
from utc.src.constants.file_system.my_file import MyFile
from utc.src.graph import Edge, Route
from utc.src.routing.base.controlled_vehicle import ControlledVehicle
from utc.src.routing.base.traffic_info import ResultInfo
from utc.src.routing.base.traffic_problem import TrafficProblem
from typing import Optional, Dict, List


class PddlResult:
    """
    Class representing PDDL result files, provides utility methods of
    obtaining back the original routes
    """
    def __init__(self, name: str, files: List[str]):
        """
        :param name: of pddl result file
        :param files: of this pddl result instance (can be multiple), full path
        :raise ValueError: if files are empty
        """
        self.name: str = name
        self.files: List[str] = files
        # Checks
        if not files:
            raise ValueError(f"Error, received empty list of files for pddl result: '{self.name}'")
        # Make sure the pddl extension is last and the files are correct
        for index, file in enumerate(files):
            assert(MyFile.file_exists(file) and MyFile.get_file_name(file).startswith(self.name))
            if not file.endswith(FileExtension.PDDL):
                files[index] = file.replace(FileExtension.PDDL, "") + FileExtension.PDDL
                assert(MyFile.rename_file(file, files[index]))
        self.info: ResultInfo = ResultInfo(self.name)

    def extract_routes(self, problem: TrafficProblem) -> Optional[Dict[str, Route]]:
        """
        :param problem: current traffic problem
        :return: Dictionary mapping vehicle id's to a valid route, intended as new segment (replacement)
        of the current one, None if error occurred
        """
        # print(f"Extracting routes for {problem.info.name}")
        paths: Optional[Dict[int, List[int]]] = self.parse_result()
        if paths is None or not paths:
            return None
        # Internal ID's mapping to original
        vehicle_abstraction: Dict[int, str] = {vehicle.internal_id: vehicle.id for vehicle in problem.vehicles.values()}
        new_paths: Dict[str, Route] = {}
        for vehicle_iid, path in paths.items():
            if vehicle_iid not in vehicle_abstraction:
                print(f"Error, received invalid vehicle internal id '{vehicle_iid}'")
                continue
            vehicle: ControlledVehicle = problem.vehicles[vehicle_abstraction[vehicle_iid]]
            original_edges: List[str] = vehicle.route.get_segment_edges(vehicle.route.get_current_segment())
            new_edges: List[Edge] = problem.network.get_edges(path)
            # Check route validity, error can happen due to killing process while result file is being written, etc.
            if not new_edges or None in new_edges or not problem.network.check_edge_sequence(new_edges):
                print(f"Error, invalid path generated for vehicle: '{vehicle.id}'")
                continue
            elif not (original_edges[0] == new_edges[0].id and original_edges[-1] == new_edges[-1].id):
                print(f"Error, mismatched starting and/or ending edge generated for vehicle: '{vehicle.id}'")
                continue
            new_paths[vehicle.id] = Route(new_edges)
        return new_paths

    def parse_result(self) -> Optional[Dict[int, List[int]]]:
        """
        Parses result files of this pddl result file, replaces results (car and route pairs) in case of multiple
        being generated for the same problem file (assumes lexicographical ordering).

        :return: Dictionary mapping vehicle id (abstract) to
        list of route id's (internal), None if file(s) could not be opened
        """
        # Checks
        if not self.files:
            return None
        paths: Dict[int, List[int]] = {}
        # Replace previous pddl result by next (assuming lexicographical ordering for better results)
        car_id, route_id = "", 0
        for file in self.files:
            curr_paths: Dict[int, List[int]] = {}
            with open(file, "r") as pddl_result:
                for line in pddl_result:
                    line = line.rstrip()
                    assert(line.startswith("(") and line.endswith(")"))
                    line = line[1:-1].split()
                    # MIP
                    if line[0].startswith("v"):
                        assert (line[2].startswith("r"))
                        car_id = int(line[0][1:])
                        route_id = int(line[2][1:])
                    # Mercury
                    else:
                        assert(line[1].startswith("v"))
                        assert (line[3].startswith("r"))
                        car_id = int(line[1][1:]) # Remove the starting symbol to obtain internal id
                        route_id = int(line[3][1:]) # Remove the starting symbol to obtain internal id
                    if car_id not in curr_paths:
                        curr_paths[car_id] = []
                    curr_paths[car_id].append(route_id)
            # Replaces keys by new ones
            paths |= curr_paths
        return paths
