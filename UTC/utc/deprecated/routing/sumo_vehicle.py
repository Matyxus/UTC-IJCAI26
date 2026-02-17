from utc.src.simulator.vehicle import Vehicle
from typing import Optional, List, Set, Tuple, Dict


class SumoRoute:
    """
    Vehicle which represents segment of route of vehicle in SUMO
    which is inside some of the regions.
    """
    def __init__(self, region_id: int, first_edge: str, first_edge_index: int, region_segment: List[str]):
        """
        :param region_id: Id (index) of visited region by this segment
        :param first_edge: Id (original) edge of the route segment
        :param first_edge_index: Index of the first edge in the whole route
        :param region_segment: Route segment to be planned (entry_edge, region_edge, ..., region_edge, exit_edge)
        """
        self.region_id: int = region_id
        self.first_edge: str = first_edge
        self.first_edge_index: int = first_edge_index
        self.region_segment: List[str] = region_segment
        self.eta: float = -1  # Eta of vehicle to arrive in the region
        self.eta_calculated: float = -1  # Time when was Eta calculated
        self.arrival: float = -1  # Real arrival time of vehicle to the region
        self.planned_segment: Optional[List[str]] = None  # Planned route segment
        self.flag: bool = False


class SumoVehicle(Vehicle):
    """
    Vehicle class to represent vehicle in SUMO.
    """
    def __init__(self, attributes: Dict[str, str], route: Tuple[str], departed: float, segments: List[SumoRoute]):
        """
        :param attributes: vehicle attributes
        :param route: whole route of vehicle (edges original id's)
        :param departed: time of vehicle departure (arrival to road network)
        :param segments: route segments which are on some of the regions
        """
        super().__init__(attributes)
        self.route: List[str] = list(route)
        self.segments: List[SumoRoute] = segments
        self.index: int = 0  # Current route segment being considered
        # Time metrics
        self.depart_time: float = departed
        self.arrival_time: float = -1

    def get_current_segment(self) -> Optional[SumoRoute]:
        """
        :return:
        """
        if not self.segments:
            print(f"Error, vehicle {self.id} has no segments!")
            return None
        return self.segments[self.index]

    def get_expected_arrival(self) -> float:
        """
        :return: Expected time of arrival of vehicle to the next region
        """
        if not self.segments:
            print(f"Error, vehicle {self.id} has no segments!")
            return -1
        return self.segments[self.index].eta_calculated + self.segments[self.index].eta

    def switch_segment(self) -> bool:
        """
        :return: True if segment can be switched, False if vehicle has no more routes in regions
        """
        if self.index + 1 >= len(self.segments):
            return False
        self.index += 1
        return True


class VehicleStats:
    """
    Statistics about vehicles
    """
    def __init__(self):
        self.departed: int = 0 # How many vehicles entered simulation
        self.added: int = 0 # How many of departed go over regions
        self.scheduled: int = 0 # How many of added were scheduled
        self.rescheduled: int = 0 # How many vehicles were re-scheduled
        self.removed: int = 0 # How many were unable to be scheduled further
        self.planned: int = 0 # How many were planned out of scheduled
        self.arrived: int = 0 # How many vehicle left simulation
        self.missed: int = 0 # How many were missed (incorrect eta) for next planning
        self.correct: int = 0 # How many planned vehicles which were assigned route (previously) arrived to region

    def reset(self) -> None:
        """
        Sets all attributes to 0

        :return: None
        """
        variables: Dict[str, int] = vars(self)
        for key in variables:
            variables[key] = 0
        return

    def __add__(self, other: 'VehicleStats') -> 'VehicleStats':
        """
        :param other: Class of vehicle statistics
        :return: Self with incremented attributes
        """
        if not isinstance(other, VehicleStats):
            raise TypeError(f"Cannot add {type(other)} into VehicleStats")
        variables: Dict[str, int] = vars(self)
        for key, value in vars(other).items():
            variables[key] += value
        return self

    def __str__(self) -> str:
        ret_val: str = f"******* Stats *******\n"
        for key, value in vars(self).items():
            ret_val += f"{key.capitalize()}: {value}\n"
        return ret_val.removesuffix("\n")


# For testing purposes
if __name__ == '__main__':
    stats: VehicleStats = VehicleStats()
    print(stats)


