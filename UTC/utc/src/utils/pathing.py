from utc.src.graph import Junction, Route, Graph, RoadNetwork, Edge
from xml.etree.ElementTree import Element
from copy import deepcopy
import time
import heapq
from collections import defaultdict, deque
from typing import Optional, List, Set, Tuple, Dict


class Pathing:
    def __init__(self, network: RoadNetwork):
        self.network = network

    def a_start(self, start_id: str, target_id: str) -> Optional[Route]:
        """
        :param start_id:
        :param target_id:
        :return:
        """
        print(f"Looking for path from {start_id} to {target_id}")
        assert(self.network.edge_exists(start_id) and self.network.edge_exists(target_id))
        start, target = self.network.get_edges([start_id, target_id])
        # -------------------------- Init --------------------------
        # priority, in_route, path (list containing all visited edges)
        queue: List[Tuple[float, Route, float, List[int]]] = []
        shortest_route: Optional[Route] = None
        destination_pos: Tuple[float, float] = self.network.get_junction(target.to_junction).get_position()
        destination_id: str = self.network.get_junction(target.to_junction).id
        # For junction n, gScore[n] is the cost of the cheapest path from start to n currently known,
        # reworked to be mapping to routes (since road-network, can be multi-graph)
        g_score: Dict[Route, float] = {}
        # Initial push
        for out_route in self.network.junctions[start.from_junction].get_out_routes():
            if out_route.edge_list[0].id == start_id:
                queue.append((0, out_route, out_route.get_length(), [start.internal_id]))
                g_score[out_route] = 0
                break
        # -------------------------- Search --------------------------
        while queue:
            priority, in_route, length, path = heapq.heappop(queue)  # Removes and returns
            # Arrived to target junction
            if in_route.get_destination() == destination_id:
                # Did not arrive by target edge
                if in_route.last_edge().id != target_id:
                    continue
                # Reconstruct path
                shortest_route = Route(self.network.get_edges(path))
                break
            for route in self.network.junctions[in_route.get_destination()].travel(in_route):
                distance, neigh_junction_id = route.traverse()
                distance += g_score[in_route]
                if distance < g_score[route]: # and not self.has_loop(route, path):
                    pos: Tuple[float, float] = self.network.junctions[neigh_junction_id].get_position()
                    g_score[route] = distance
                    heapq.heappush(queue, (
                        distance + self.coord_distance(destination_pos, pos), route,
                        distance, path + route.get_edge_ids(True)
                    ))
        return shortest_route
    
    
    def dijkstra(self, in_edge: str, out_edge: str) -> Optional[Route]:
        """
        :param in_edge: Starting edge id
        :param out_edge: Ending edge id
        :return: Shortest path from 'in_edge' to 'out_edge'
        """
        # --- Init ---
        start_edge, exit_edge = self.network.get_edges([in_edge, out_edge])
        start_route, exit_route = self.network.get_routes([start_edge.internal_id, exit_edge.internal_id])
        assert(start_route.edge_list[0] == start_edge and exit_route.edge_list[0] == exit_edge)
        costs: Dict[str, float] = {route.id: float("inf") for route in self.network.get_routes_list()}
        prev: Dict[str, str] = {}
        queue: List[Tuple[float, Junction, Route]] = []
        costs[start_route.id] = 0
        heapq.heappush(queue, (0, self.network.get_junction(start_edge.to_junction), start_route))
        # --- Path finding ---
        while queue:
            total_cost, junction, in_route = heapq.heappop(queue)
            # Found target, reconstruct route
            if in_route.id == exit_route.id:
                path: List[str] = [exit_route.id]
                while path[-1] in prev and costs[path[-1]] != float("inf"):
                    path.append(prev[path[-1]])
                path.reverse()
                edges = []
                for route_id in path:
                    edges += self.network.get_route(route_id).edge_list
                assert(self.network.check_edge_sequence(edges))
                # print(f"Found path with cost: {total_cost} - {[edge.id for edge in edges]}")
                return Route(edges)
            for out_route in junction.travel(in_route):
                cost, dest = out_route.travel()
                cost += total_cost
                if cost < costs[out_route.id]:
                    prev[out_route.id] = in_route.id
                    costs[out_route.id] = cost
                    heapq.heappush(queue, (cost, self.network.get_junction(dest), out_route))
        print(f"Unable to find path between: {in_edge, out_edge} !")
        return None

    # # noinspection PyMethodMayBeStatic
    # def has_loop(self, route: Route, path: List[int]) -> bool:
    #     """
    #     :param route: currently considered route
    #     :param path: all visited edges on path
    #     :return: True if there is overlap between route edges and visited edges, False otherwise
    #     """
    #     return route.edge_list[0].internal_id in path

    # noinspection PyMethodMayBeStatic
    def coord_distance(self, point_a: Tuple[float, float], point_b: Tuple[float, float]) -> float:
        """
        :param point_a: first point
        :param point_b: second point
        :return: absolute distance between points (3 decimal precision)
        """
        return round((((point_a[0] - point_b[0]) ** 2) + ((point_a[1] - point_b[1]) ** 2)) ** 0.5, 3)



if __name__ == '__main__':
    graph: Graph = Graph(RoadNetwork())
    graph.loader.load_map("DCC_central")
    pathing: Pathing = Pathing(graph.road_network)
    dijkstra_path = pathing.dijkstra("52881220#0", "22962975#2")
    astar_path = pathing.a_start("52881220#0", "22962975#2")








