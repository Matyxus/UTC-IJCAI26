from utc.src.graph import RoadNetwork, Junction, Edge, Route, Graph
from utc.src.routing.base.controlled_vehicle import ControlledVehicle
import heapq
import time
from typing import Optional, List, Set, Tuple, Dict


class DUO:
    """
    Class dealing with decentralized routing approach, i.e. other vehicles are not taken into account
    """
    def __init__(self, graph: Graph, sub_graphs: List[Graph] = None):
        """
        :param graph: the graph on which routing takes place
        :param sub_graphs: sub-graphs (controlled regions) of road network
        """
        self.graph: Graph = graph
        self.sub_graphs: Optional[List[Graph]] = sub_graphs
        print("Successfully initialized DUO routing")

    def route_vehicles(self, vehicles: List[ControlledVehicle]) -> Tuple[List[Optional[Route]], float]:
        """
        :param vehicles: list of vehicles scheduled for routing
        :return: List of new routes for vehicles current segments (some can be invalid - None) and time taken
        """
        now: float = time.time()
        return [self.route_vehicle(vehicle) for vehicle in vehicles], round(time.time() - now, 3)

    def route_vehicle(self, vehicle: ControlledVehicle) -> Optional[Route]:
        """
        :param vehicle: current vehicle scheduled for routing
        :return: New route for vehicle's current segment, None if it does not exist
        """
        edges: List[str] = vehicle.route.get_segment_edges(vehicle.route.get_current_segment())
        region_id: int = vehicle.route.get_current_segment().region_id
        if region_id == -1:
            return self.dijkstra(edges[0], edges[-1], self.graph.road_network)
        elif self.sub_graphs is None or not self.sub_graphs or region_id >= len(self.sub_graphs):
            print(f"Error, unable to route vehicle {vehicle.id} with DUO, missing sub-graph!")
            return None
        # Else
        return self.dijkstra(edges[0], edges[-1], self.sub_graphs[region_id].road_network)

    # ---------------------------------------- Routing ----------------------------------------

    def dijkstra(self, in_edge: str, out_edge: str, network: RoadNetwork) -> Optional[Route]:
        """
        Computes the fastest (in terms of travel time) route for vehicle.

        :param in_edge: incoming edge
        :param out_edge: outgoing edge
        :param network: road network on which the computation takes place
        :return: Fastest travel time route, None if it does not exist
        """
        # ---------- Checks ----------
        if not network.edge_exists(in_edge) or not network.edge_exists(out_edge):
            return None
        start_edge, exit_edge = network.get_edges([in_edge, out_edge])
        if not network.route_exists(start_edge.internal_id) or not network.route_exists(exit_edge.internal_id):
            return None
        start_route, exit_route = network.get_routes([start_edge.internal_id, exit_edge.internal_id])
        assert(start_route.edge_list[0].id == in_edge and exit_route.edge_list[-1].id == out_edge)
        # ---------- Init ----------
        costs: Dict[str, float] = {start_route.id: 0}
        prev: Dict[Route, Optional[Route]] = {start_route: None}
        queue: List[Tuple[float, Junction, Route]] = []
        heapq.heappush(queue, (0, network.get_junction(start_route.get_destination()), start_route))
        # ---------- Path finding ----------
        while queue:
            total_cost, junction, in_route = heapq.heappop(queue)
            # Skip already explored
            # if costs[in_route.id] != total_cost:
            #     continue
            # Reached target, reconstruct route
            if in_route.edge_list[-1].id == out_edge:
                edges: List[Edge] = []
                current = in_route
                while current is not None:
                    edges += current.edge_list
                    current = prev[current]
                edges.reverse()
                assert(network.check_edge_sequence(edges))
                # Special case of vehicle only driving over one edge
                if total_cost == 0:
                    assert(in_edge == out_edge)
                return Route(edges)

            for out_route in junction.travel(in_route):
                cost, dest = out_route.travel()
                assert(cost > 0)
                cost += total_cost
                if cost < costs.get(out_route.id, float("inf")):
                    prev[out_route] = in_route
                    costs[out_route.id] = cost
                    heapq.heappush(queue, (cost, network.get_junction(dest), out_route))
        print(f"Unable to find path between: {in_edge, out_edge} !")
        return None

