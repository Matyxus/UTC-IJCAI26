from dataclasses import dataclass, asdict
from typing import Optional


@dataclass
class VehicleInfo:
    """ Class holding information about vehicles during routing process """
    total: int = 0          # Total number of vehicles considered for routing
    scheduled: int = 0      # Total number of vehicles scheduled for routing
    routed: int = 0         # How many vehicles were routed by solver
    short_route: int = 0    # How many were discarded because of short route (<3 edges)
    invalid_route: int = 0  # No route found (self loop), or only 1 route found

    def __add__(self, other: 'VehicleInfo') -> 'VehicleInfo':
        """
        :param other: vehicle info class
        :return: 'self' with added values from 'other'
        """
        self.total += other.total
        self.scheduled += other.scheduled
        self.routed += other.routed
        self.short_route += other.short_route
        self.invalid_route += other.invalid_route
        return self


@dataclass
class NetworkInfo:
    """ Class holding information about road network construction for specific TrafficProblem """
    time: float = 0         # Time taken (seconds) to generate road network
    routes: int = 0     # Total number of routes (in the network)
    junctions: int = 0  # Total number of junctions (in the network, does not account for splitting)

    def __add__(self, other: 'NetworkInfo') -> 'NetworkInfo':
        """
        :param other: problem info class
        :return: 'self' with added values from 'other'
        """
        self.time += other.time
        self.routes += other.routes
        self.junctions += other.junctions
        return self


@dataclass
class ResultInfo:
    """ Class holding information about pddl result during routing process """
    name: str             # Name of pddl result file
    cost: int = 0         # Plan cost
    plans: int = 0        # How many plan files were generated for this problem
    timeout: float = 0.0  # How much time did solver use (3 digit precision)

    def __add__(self, other: 'ResultInfo') -> 'ResultInfo':
        """
        :param other: result info class
        :return: 'self' with added values from 'other'
        """
        self.cost += other.cost
        self.plans += other.plans
        self.timeout += other.timeout
        return self


class EpisodeInfo:
    """
    Class representing information of entire traffic episode,
    combining network, result and vehicle information into a single class.
    """
    def __init__(
            self, name: str, vehicle_info: VehicleInfo,
            network_info: NetworkInfo, result_info: Optional[ResultInfo] = None
        ):
        """
        :param name: name of traffic episode (unique)
        :param vehicle_info: information about vehicles
        :param network_info: information about road network
        :param result_info information about pddl result (can be None if it was not generated)
        """
        self.name: str = name
        self.vehicle_info: VehicleInfo = vehicle_info
        self.network_info: NetworkInfo = network_info
        self.result_info: Optional[ResultInfo] = result_info

    def is_valid(self) -> bool:
        """
        :return:
        """
        return self.id >= 0 and self.result_info is not None

    def to_dict(self) -> dict:
        """
        :return: dictionary representation of pddl episode info
        """
        return {
            f"episode_{self.name}": {
                "network": asdict(self.network_info),
                "vehicle": asdict(self.vehicle_info),
                "result": None if self.result_info is None else asdict(self.result_info)
            }
        }

    def __add__(self, other: 'EpisodeInfo') -> 'EpisodeInfo':
        """
        :param other: episode info class
        :return: self with values added from other episode
        """
        assert(self.is_valid() and other.is_valid())
        self.vehicle_info += other.vehicle_info
        self.network_info += other.network_info
        if self.is_valid() and other.is_valid():
            self.result_info += other.result_info
        return self


