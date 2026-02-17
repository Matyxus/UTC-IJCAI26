from utc.src.routing.base.controlled_vehicle import ControlledVehicle
from utc.src.routing.base.traffic_info import VehicleInfo, NetworkInfo, EpisodeInfo
from utc.src.graph import RoadNetwork
from typing import Optional, List, Dict, FrozenSet


class TrafficProblem:
    """
    Class representing traffic routing problem, contains vehicles,
    road network constructed for them and their sub-graphs.
    Holds additional information related statistics.
    """

    def __init__(self, name: str, vehicles: List[ControlledVehicle], network: Optional[RoadNetwork] = None):
        """
        :param name: name of the problem (to differentiate between others)
        :param vehicles: container of vehicles which will be routed
        :param network: road network on which vehicle will be routed
        """
        self.network: Optional[RoadNetwork] = network # Road network of vehicles
        self.vehicles: Dict[str, ControlledVehicle] = {  # Vehicles of routing problem
            vehicle.id: vehicle for vehicle in vehicles
        }
        self.sub_graphs: Dict[str, FrozenSet[int]] = {} # Custom vehicle sub-graphs (for 'allowed' predicate)
        self.info: EpisodeInfo = EpisodeInfo(name, VehicleInfo(), NetworkInfo())

    # ------------------------------------ Utils ------------------------------------

    def is_valid(self) -> bool:
        """
        :return: True if this class instance is valid PDDL problem, false otherwise
        """
        return self.network is not None and self.vehicles

    def free(self) -> None:
        """
        Frees memory of traffic episode, i.e. the network, vehicles, etc.,
        only information about traffic episode will remain

        :return: None
        """
        self.network = None
        self.vehicles.clear()
        self.sub_graphs.clear()
        return
