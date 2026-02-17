from utc.src.simulator.vehicle import Vehicle
from utc.src.graph import Route
from typing import Optional, Tuple, List, FrozenSet


class Segment:
    """ Class representing segments of routes in vehicle routing """
    def __init__(self, start_i: int, end_i: int, region_id: int = -1):
        """
        :param start_i: start index of segment
        :param end_i: last index of segment
        :param region_id: id of controlled region (-1 means no region, i.e. outside)
        """
        assert(0 <= start_i < end_i)
        self.indexes: Tuple[int, int] = (start_i, end_i)
        self.region_id = region_id
        # List of new edges produced by routing solvers (DUO/DSO)
        self.routed_segment: List[str] = []
        self.routed_by: str = ""
        # Estimated time of arrival in the segment's end (0 is default = unknown)
        self.eta: float = 0

class ControlledRoute:
    """ Class representing routes in vehicle routing """
    def __init__(self, route_id: str, edges: List[str]):
        """
        :param route_id: route identifier
        :param edges: list of edge id's (original) forming route
        """
        self.id: str = route_id
        # self.original: List[str] = edges
        self.edges: List[str] = edges # Edges of route, dynamically updated
        self.segments: List[Segment] = []
        self.current: int = 0 # Index of current segment

    def update_current_segment(self, new_edges: List[str], routed_by: str) -> bool:
        """
        This update occurs after dynamic route assigment happened in simulation (TraCI),
        edges of route are also updated, as are subsequent segment indexes of route.

        :param new_edges: new list of edges to replace current segment
        :param routed_by: type of routing used to generate new segment
        :return: True on success, False otherwise
        """
        segment: Segment = self.get_current_segment()
        original_edges: List[str] = self.get_segment_edges(segment)
        # Validity check
        if original_edges[0] != new_edges[0] or original_edges[-1] != new_edges[-1]:
            print(f"Error, invalid segment given for updating to route: {self.id}")
            return False
        segment.routed_segment = new_edges
        segment.routed_by = routed_by
        size_diff: int = len(new_edges) - (segment.indexes[1] - segment.indexes[0])
        # Replace edges
        self.edges[segment.indexes[0]:segment.indexes[1]] = new_edges
        # Adjust indexes
        if size_diff != 0:
            segment.indexes = (segment.indexes[0], segment.indexes[1] + size_diff)
            for seg in self.segments[(self.current+1):]:
                seg.indexes = (seg.indexes[0] + size_diff, seg.indexes[1] + size_diff)
        return True

    # --------------------------------- Getters ---------------------------------

    def get_segment_edges(self, segment: Segment) -> Optional[List[str]]:
        """
        :param segment: segment of which to get edges
        :return: List of edges forming given segment, None if segment is not from this route
        """
        start, end = segment.indexes
        if start < 0 or end > len(self.edges):
            print(f"Error, segment is not from route {self.id} !")
            return None
        return self.edges[start:end]

    def get_current_segment(self) -> Segment:
        """
        :return: current segment
        """
        return self.segments[self.current]

    def get_next_region_segment(self) -> Optional[Segment]:
        """
        :return: Next segment in controlled region, None if there is not another one
        """
        for i in range(self.current, len(self.segments)):
            if self.segments[i].region_id != -1:
                return self.segments[i]
        return None

class ControlledVehicle(Vehicle):
    """ Class representing vehicles in vehicle routing """
    def __init__(self, vehicle: Vehicle, edges: List[str]):
        """
        :param vehicle: representation of SUMO's vehicle
        :param edges: edges of vehicle's route
        """
        super().__init__(vehicle.attributes, vehicle.internal_id)
        self.route: ControlledRoute = ControlledRoute(vehicle.get_attribute("route"), edges)

    def switch_segment(self) -> bool:
        """
        :return: True if route segment can be switched, False if vehicle is on its last segment
        """
        if self.route.current + 1 >= len(self.route.segments):
            return False
        self.route.current += 1
        return True



