from utc.src.routing.base.traffic_problem import TrafficProblem, ControlledVehicle, VehicleInfo
from utc.src.routing.routing_options import NetworkBuilderOptions
from utc.src.routing.traffic.cache import Cache
from utc.src.graph import Graph, RoadNetwork, Route, Junction, Edge
from utc.src.clustering.similarity.similarity_clustering import SimilarityClustering
from typing import Optional, List, Dict, Set, FrozenSet


class NetworkBuilder:
    """
    Class simplifying and build road network for routing solvers
    """
    def __init__(self, graph: Graph, options: NetworkBuilderOptions):
        """
        :param graph: on which re-routing takes place
        :parm options: network builder options
        """
        assert(None not in (graph, options))
        self.graph: Graph = graph
        self.options: NetworkBuilderOptions = options
        self.sim_clustering: Optional[SimilarityClustering] = (
            None if options.dbscan is None else
            SimilarityClustering(options.dbscan)
        )
        self.cache: Cache = Cache(options.cache_size)  # Memory of previously constructed sub-graphs

    # ------------------------------------------ Network construction ------------------------------------------

    def build_network(self, problem: TrafficProblem) -> bool:
        """
        Builds (and assigns) custom road network for vehicle of traffic problem

        :param problem: Current traffic problem (episode)
        :return: True on success, False otherwise
        """
        print(f"Building sub-graph for {len(problem.vehicles)} vehicles")
        # Checks
        if not problem.vehicles:
            print("Invalid vehicles, mapping is empty, cannot construct road network!")
            return False
        edges: Set[int] = set()
        # For all vehicle generate corresponding sub-graph (all found edges)
        for vehicle in problem.vehicles.values():
            sub_graph : Optional[FrozenSet[int]] = self.generate_graph(vehicle, problem.info.vehicle_info)
            if sub_graph is not None:
                edges |= sub_graph
                problem.sub_graphs[vehicle.id] = sub_graph
        # TODO Check if all simplification is turned off (parameter)
        if self.options.topka is not None:
            problem.info.vehicle_info.scheduled = len(problem.sub_graphs)
            print(f"Built sub-graphs for {len(problem.sub_graphs)}/{len(problem.vehicles)} vehicles")
            if edges: # Combine parts to build graph (allowed-subgraph unique to vehicle)
                problem.network = self.graph.sub_graph.create_sub_graph(self.graph.road_network.get_edges(edges))
        else: # Simplifying is turned off, use whole network
            problem.network = self.graph.road_network
        return problem.network is not None

    # ------------------------------------------ Route generation ------------------------------------------

    def generate_graph(self, vehicle: ControlledVehicle, info: VehicleInfo) -> Optional[FrozenSet[int]]:
        """
        :param vehicle: class holding attributes of vehicle
        :param info: information about vehicles
        :return: Subgraph as set of edge (internal) id's forming it
        """
        # Check if vehicle has valid route
        if not self.check_route(vehicle, info):
            return None
        edges: List[Edge] = self.graph.road_network.get_edges(
            vehicle.route.get_segment_edges(vehicle.route.get_current_segment())
        )
        # Check if we already generated such sub-graph, if yes return it (can be also 'None')
        if self.cache.has_mapping(edges[0].internal_id, edges[-1].internal_id):
            # print(f"Mapping for vehicle exists ...")
            return self.cache.get_mapping(edges[0].internal_id, edges[-1].internal_id)
        # # Extract second (i.e. we start on the edge) and ending junctions of route
        # start_junction: Junction = self.graph.road_network.get_junction(edges[0].to_junction)
        # end_junction: Junction = self.graph.road_network.get_junction(edges[-1].to_junction)
        # Apply TopKA*
        if self.options.topka is not None:
            # Find routes
            routes: List[Route] = self.graph.path_finder.top_k_a_star2(
                edges[0].id, edges[-1].id,
                c=self.options.topka.c, k=self.options.topka.k
            )
            # Invalid routes, or only shortest path was found
            if routes is None or not routes or len(routes) == 1:
                # print(f"TopKA* did not find any alternative routes for vehicle: {vehicle.id}")
                info.invalid_route += 1
                self.cache.invalid.add((edges[0].internal_id, edges[-1].internal_id))
                return None
            # Apply clustering on routes
            if self.sim_clustering is not None:
                indexes: Optional[List[int]] = self.sim_clustering.calculate(routes)
                if indexes is not None and indexes:
                    # print(f"Applied DBSCAN on routes ...")
                    routes = [routes[index] for index in indexes]
            return self.cache.save_mapping(edges[0].internal_id, edges[-1].internal_id, routes)
        # Other techniques ...
        return None

    def check_route(self, vehicle: ControlledVehicle, info: VehicleInfo) -> bool:
        """
        :param vehicle: class representing vehicle
        :param info: information about vehicles
        :return: True if vehicle has valid route on graph, False otherwise
        """
        # print(f"------ Routing vehicle: {pddl_vehicle.vehicle.get_attribute('id')} ------")
        # Check vehicles current segment (the one being routed), all must be on graph
        original_edges: List[str] = vehicle.route.get_segment_edges(vehicle.route.get_current_segment())
        edges = self.graph.road_network.get_edges(original_edges, message=False)
        if not edges or None in edges:
            info.invalid_route += 1
            print(f"Vehicle: {vehicle.id} does not drive on network !")
            quit()
            return False
        elif len(edges) < 3:
            info.short_route += 1
            # print(f"Vehicle: {vehicle.id} route is too short, will only be accounted for in capacity!")
            return False
        # TODO quick-hack
        elif vehicle.route.get_current_segment().eta < 3:
            # print(f"Vehicle: {vehicle.id} eta is too low, will only be accounted for in capacity!")
            return False
        return self.graph.road_network.check_edge_sequence(edges)

