from typing import List, Tuple, Dict, Optional, Set
from utc.src.graph import Graph, RoadNetwork, Junction, Route, Edge
import heapq
import matplotlib.pyplot as plt
import numpy as np
import time
import random

def forward_dijsktra(start: Junction, length: float, graph: Graph) -> Dict[int, float]:
    now: float = time.time()
    distances: Dict[int, float] = {route.internal_id: float("inf") for route in graph.road_network.get_routes_list()}
    queue: List[Tuple[float, Junction, Route]] = []
    # Start
    for out_route in start.travel(None):
        dist, dest = out_route.traverse()
        distances[out_route.internal_id] = dist
        heapq.heappush(queue, (dist, graph.road_network.get_junction(dest), out_route))
    # Exploration
    iters: int = 0
    while queue:
        distance, junction, in_route = heapq.heappop(queue)
        iters += 1
        for out_route in junction.travel(in_route):
            dist, dest = out_route.traverse()
            dist += distance
            if dist < distances[out_route.internal_id] and dist < length:
                distances[out_route.internal_id] = dist
                heapq.heappush(queue, (dist, graph.road_network.get_junction(dest), out_route))
    print(f"Expanded {iters} iterations in: {round(time.time()-now, 3)} seconds")
    return {k:round(v, 3) for k, v in distances.items()}


def backward_dijsktra(start: Junction, length: float, graph: Graph) -> Dict[int, float]:
    now: float = time.time()
    distances: Dict[int, float] = {route.internal_id: float("inf") for route in graph.road_network.get_routes_list()}
    queue: List[Tuple[float, Junction, Route]] = []
    connections: Dict[int, Set[int]] = graph.road_network.get_edges_connections(internal=True)
    # Start
    for in_route in start.get_in_routes():
        dist, dest = sum(edge.length for edge in in_route.edge_list), in_route.get_start()
        distances[in_route.internal_id] = dist
        heapq.heappush(queue, (dist, graph.road_network.get_junction(dest), in_route))
    # Exploration
    iters: int = 0
    while queue:
        distance, junction, to_route = heapq.heappop(queue)
        iters += 1
        if to_route.internal_id not in connections:
            continue
        for in_route in connections[to_route.internal_id]:
            route: Route = graph.road_network.get_route(in_route)
            dist, dest = sum(edge.length for edge in route.edge_list), route.get_start()
            dist += distance
            if dist < distances[in_route] and dist < length:
                distances[in_route] = dist
                heapq.heappush(queue, (dist, graph.road_network.get_junction(dest), route))
    print(f"Expanded {iters} iterations in: {round(time.time()-now, 3)} seconds")
    return {k:round(v, 3) for k, v in distances.items()}


if __name__ == '__main__':
    # 389620 cluster_2374841823_3238182641_4463318859
    random.seed(42)
    c: float = 1.25
    graph = Graph(RoadNetwork())
    assert(graph.loader.load_map("lust_central"))
    from_junction, to_junction = graph.road_network.get_junction("-18220"), graph.road_network.get_junction("-2258")
    assert(from_junction is not None and to_junction is not None)
    shortest_path: Route = graph.path_finder.a_star(from_junction.id, to_junction.id)[1]
    assert(shortest_path is not None)
    length: float = shortest_path.traverse()[0]
    limit: float = round(length * c, 3)
    print(f"Shortest path length: {length}, limit: {limit}")
    forward = forward_dijsktra(from_junction, limit, graph)
    backward = backward_dijsktra(to_junction, limit, graph)
    start_edge, end_edge = graph.road_network.get_edges(["--31594#0", "--31818#11"])
    print(forward[end_edge.internal_id], backward[start_edge.internal_id])
    limited_edges: List[Edge] = []
    for edge in graph.road_network.get_edge_list():
        if (forward[edge.internal_id] + backward[edge.internal_id]) < limit:
            limited_edges.append(edge)
    print(f"Found {len(limited_edges)} limited edges")
    fig, ax = graph.display.initialize_plot()
    graph.display.render_edges(ax, graph.road_network.get_edge_list())
    graph.display.render_edges(ax, limited_edges, colors="blue")
    graph.display.render_routes(ax, [shortest_path], colors="red")
    graph.display.show_plot(ax)
    # sg: Graph = Graph(graph.sub_graph.create_sub_graph(limited_edges))
    # sg.display.plot_graph(False)
    # fig, ax = graph.display.initialize_plot()
    # graph.display.render_edges(ax, graph.road_network.get_edge_list())
    # graph.display.render_junctions(ax, [from_junction, to_junction], colors=["green", "blue"])
    # graph.display.render_routes(ax, [shortest_path], colors="red")
    # graph.display.show_plot(ax)
