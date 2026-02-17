from utc.src.constants.static.pddl_constants import NetworkCapacity
from utc.src.routing.base.traffic_problem import TrafficProblem
from utc.src.routing.pddl.base.pddl_problem import PddlProblem
from utc.src.graph import Route, Junction
from typing import Dict, List, Set


class NetworkDomain:
    """
    Class holding representation of road networks for '.pddl' problem files
    """
    def __init__(self, dynamic_cost: bool = False):
        """
        :param dynamic_cost: True if dynamic cost should be used for route cost, False otherwise
        """
        self.dynamic_cost = dynamic_cost
        self.use_object_group: str = "use"
        self.junction_group_name: str = "junction"
        self.route_group_name: str = "road"

    def process_graph(self, pddl_problem: PddlProblem, traffic_problem: TrafficProblem) -> bool:
        """
        Creates basic pddl representation of graph\n
        ':init' -> (connected junction_id route_id junction_id),\n
        adds id's of junction to group: junction,\n
        id's of routes to group: road\n
        -> ':object' -> j{junction_id}, ..., - junction\n
        -> ':object' -> r{route_id}, ..., - road

        :param pddl_problem: instance of pddl problem
        :param traffic_problem: instance of traffic problem
        :return: True on success, false otherwise
        """
        print("Converting road network to pddl representation ...")
        if not self.process_connections(pddl_problem, traffic_problem):
            return False
        if not self.process_routes(pddl_problem, traffic_problem):
            return False
        elif not self.generate_allowed_predicate(pddl_problem, traffic_problem):
            return False
        # print("Successfully converted road network into its pddl representation")
        return True

    def process_connections(self, problem: PddlProblem, traffic_problem: TrafficProblem) -> bool:
        """
        Processes connections between junctions and routes,
        adds all junctions (including split ones) to pddl problem.
        Also defined 'changeable' predicate (which vehicles use to change
        between split junctions in case its their starting/ending one).

        :param problem: instance of pddl problem
        :param traffic_problem: instance of traffic problem
        :return: True on success, false otherwise
        """
        # print("Transforming graph into pddl ...")
        # Mapping of routes and their starting/ending junctions
        connections: Dict[int, List[Set[str], Set[str]]] = {
            # route_id: ({starting junctions}, {ending junctions})
            route.get_id(True): [set(), set()] for route in traffic_problem.network.routes.values()
        }
        # -------------- Add junctions --------------
        for junction in traffic_problem.network.junctions.values():
            junction_ids: Set[str] = set()
            for route_id, (in_junctions, out_junctions) in self.decompose_junction(junction, True).items():
                connections[route_id][0] |= in_junctions
                connections[route_id][1] |= out_junctions
                junction_ids |= (in_junctions | out_junctions)
            assert(f"j{junction.get_id(True)}" in junction_ids)
            # Add junctions to the objects
            for junction_name in junction_ids:
                problem.add_object(self.junction_group_name, junction_name)
        traffic_problem.info.junctions = len(traffic_problem.network.junctions)
        # print(connections)
        # quit()
        # -------------- Add artificial junctions for vehicles --------------
        for vehicle in traffic_problem.vehicles.values():
            if vehicle.id not in traffic_problem.sub_graphs:
                continue
            # For each vehicle add artificial starting an ending junctions,
            # these junctions only have their valid starting & ending routes of vehicle
            starting_junction = f"js{vehicle.internal_id}"
            ending_junction = f"je{vehicle.internal_id}"
            problem.add_object(self.junction_group_name, starting_junction)
            problem.add_object(self.junction_group_name, ending_junction)
            segment_edges: List[str] = vehicle.route.get_segment_edges(vehicle.route.get_current_segment())
            assert(traffic_problem.network.edge_exists(segment_edges[0]))
            assert(traffic_problem.network.edge_exists(segment_edges[-1]))
            starting_edge, ending_edge = traffic_problem.network.get_edges([segment_edges[0], segment_edges[-1]])
            starting_route, ending_route = starting_edge.internal_id, ending_edge.internal_id
            assert(traffic_problem.network.get_route(starting_route).edge_list[0] == starting_edge)
            assert(traffic_problem.network.get_route(ending_route).edge_list[-1] == ending_edge)
            assert(starting_route in connections and ending_route in connections)
            # Add connection between artificial starting junction, this route and destination
            for out_junction in connections[starting_route][1]:
                problem.add_init_state(f"(connected {starting_junction} r{starting_route} {out_junction})")
            # Add connection between artificial ending junction, this route and starting junctions
            for in_junction in connections[ending_route][0]:
                problem.add_init_state(f"(connected {in_junction} r{ending_route} {ending_junction})")
        # -------------- Add connections --------------
        for route_id, (start_junctions, end_junctions) in connections.items():
            # Add all combinations of connections between split junctions
            for start_junction in start_junctions:
                for end_junction in end_junctions:
                    problem.add_init_state(f"(connected {start_junction} r{route_id} {end_junction})")
        return True

    def process_routes(self,  problem: PddlProblem, traffic_problem: TrafficProblem) -> bool:
        """
        Adds predicates related to routes, e.g.: object definition, capacity, penalization, 'use' predicate, length

        :param problem: instance of pddl problem
        :return: True on success, false otherwise
        """
        # print("Transforming routes into pddl")
        #  --------------- Extend network ---------------
        # The vehicle we are not routing will be used to lower the capacity of edges they drive over
        occupied: Dict[str, int] = {}
        for vehicle in traffic_problem.vehicles.values():
            if vehicle.id in traffic_problem.sub_graphs:
                continue
            for edge in vehicle.route.get_segment_edges(vehicle.route.get_current_segment()):
                if edge not in occupied:
                    occupied[edge] = 0
                occupied[edge] += 1
        # Add predicates: 'connected', 'length', 'use', 'cap', 'using', 'light, medium, heavy'
        max_capacity: int = 0
        for route in traffic_problem.network.routes.values():
            problem.add_object(self.route_group_name, f"r{route.get_id(True)}")
            capacity: int = route.get_capacity()
            assert (capacity > 0)
            max_capacity = max(max_capacity, capacity)
            # Route penalization
            for predicate in self.add_penalization(route):
                problem.add_init_state(predicate)
            # Route capacity thresholds
            for predicate in self.add_thresholds(route, capacity):
                problem.add_init_state(predicate)
            # Maximum capacity (after it becomes congested)
            problem.add_init_state(f"(cap r{route.get_id(True)} use{capacity})")
            # Check current number of cars on route
            vehicle_count: int = 0
            if occupied:
                # Check how many edges does route have in common with occupied edges, add vehicles
                for edge_id in (occupied.keys() & set(route.get_edge_ids())):
                    vehicle_count += occupied[edge_id]
                # Maximal amount of vehicles cannot surpass capacity
                vehicle_count = min(vehicle_count, capacity)
            # Add predicate with the current usage of road
            problem.add_init_state(f"(using r{route.get_id(True)} use{vehicle_count})")
        # Add 'use', 'next' predicate (to calculate how many cars are on road)
        for i in range(max_capacity):
            problem.add_init_state(f"(next use{i} use{i + 1})")
            problem.add_object(self.use_object_group, f"use{i}")
        problem.add_object(self.use_object_group, f"use{max_capacity}")
        # problem.info.routes = len(problem.network.routes)
        return True

    def generate_allowed_predicate(self,  problem: PddlProblem, traffic_problem: TrafficProblem) -> bool:
        """
        Adds allowed predicate determining if road can be used on a way to destination by vehicle

        :param problem: instance of pddl problem
        :return: True on success, false otherwise
        """
        # print("Generating allowed predicate")
        # For each vehicle, set allowed predicate to routes it is allowed to use (the vehicle's subgraph)
        for vehicle in traffic_problem.vehicles.values():
            if vehicle.id not in traffic_problem.sub_graphs:
                continue
            for route_id in traffic_problem.sub_graphs[vehicle.id]:
                problem.add_init_state(f"(allowed v{vehicle.internal_id} r{route_id})")
        # print("Finished generating allowed predicate for sub-graphs")
        return True
    # ---------------------------------------- Utils ----------------------------------------

    def decompose_junction(self, junction: Junction, split: bool = True) -> Dict[int, List[Set[str]]]:
        """
        Decomposes junctions, avoid problems with 'allowed' predicate which occurs when it matters how
        we arrived into junction, if junction contains multiple incoming and out-going routes and among those
        not all incoming are able to reach all out-going, then we separate such junctions into multiple.\n
        Example:\n
        A -> B -> C, B <-> D, where we cannot use D -> B -> D, even if its 'allowed' by predicate,
        will get split into: A -> B -> C, D -> B' -> C, A -> B -> D

        :param split: True if junction should be split (if needed) false otherwise
        :param junction: junction of road network
        :return: List of predicates representing this junctions
        """
        num_out: int = len(set(junction.get_out_routes()))
        routes_mapping: Dict[int, List[Set[str]]] = {
            # route_id: ({starting junctions}, {ending junctions})
            route.get_id(True): [set(), set()] for route in junction.get_routes()
        }
        junction_id: str = f"j{junction.get_id(True)}"
        split_count: int = 0
        connection_added: bool = False
        full_connection: bool = any([len(out_routes) == num_out for out_routes in junction.connections.values()])
        is_ending: bool = junction.is_ending()
        for incoming_route, out_routes in junction.connections.items():
            # Ending connection, can be added to original junction (not split)
            if not out_routes:
                routes_mapping[incoming_route.get_id(True)][1].add(junction_id)
            else:  # Normal connection
                split_id: str = junction_id
                # We need to split this connection to new junction, cannot map to all out-going routes
                if (len(out_routes) != num_out or is_ending) and split:
                    # print(f"Splitting connection, route: {incoming_route} does not connect to all out routes")
                    # Create new junction only for the next split (or if there is connection with full mapping)
                    if connection_added or full_connection:
                        split_id = f"j{junction.get_id(True)}s{split_count}"
                        split_count += 1
                # Add connection (only when its not starting connections)
                if incoming_route is not None:
                    routes_mapping[incoming_route.get_id(True)][1].add(split_id)
                for out_route in out_routes:
                    routes_mapping[out_route.get_id(True)][0].add(split_id)
                # We added connection to original junction
                connection_added = True
        return routes_mapping

    # noinspection PyMethodMayBeStatic
    def get_thresholds(self, capacity: int) -> Dict[str, int]:
        """
        :param capacity: of road
        :return: Mapping of traffic density to number of cars
        """
        # At least one car has to be on as light-traffic (minimum)
        light_cap: int = max(round(capacity * 0.35), 1)
        medium_cap: int = round(capacity * 0.4)
        heavy_cap: int = capacity - light_cap - medium_cap
        if capacity - light_cap < 0:
            medium_cap = 0
            heavy_cap = 0
        elif capacity - light_cap - medium_cap < 0:
            heavy_cap = 0
        ret_val: Dict[str, int] = {
            "light": light_cap,
            "medium": medium_cap,
            "heavy": heavy_cap,
        }
        assert (light_cap + medium_cap + heavy_cap == capacity)
        return ret_val

    def add_thresholds(self, route: Route, capacity: int) -> List[str]:
        """
        :param route: to be calculated
        :param capacity: route capacity
        :return: List of predicates representing capacity of each congestion type (light/medium/heavy)
        """
        # Default for every route (-> 0 cars on road)
        route_id: str = f"r{route.get_id(True)}"
        predicates: List[str] = [f"(light {route_id} use0)"]
        index: int = 1
        for density_type, density in self.get_thresholds(capacity).items():
            for _ in range(density):
                predicates.append(f"({density_type} {route_id} use{index})")
                index += 1
        return predicates

    def add_penalization(self, route: Route) -> List[str]:
        """
        :param route: to be calculated
        :return: List of predicates representing penalization based on route congestion
        """
        assert(len(route.edge_list) == 1)
        if not self.dynamic_cost:
            cost: float = route.get_average_traveling_time()
        else:
            cost: float = min(route.get_travel_time(), 5000) #
        cost = max(cost, 1)
        assert (cost >= 1)
        route_id: str = f"r{route.get_id(True)}"
        return [
            f"(= (length-light {route_id}) {int(cost * NetworkCapacity.LIGHT_CAPACITY_MULTIPLIER)})",
            f"(= (length-medium {route_id}) {int(cost * NetworkCapacity.MEDIUM_CAPACITY_MULTIPLIER)})",
            f"(= (length-heavy {route_id}) {int(cost * NetworkCapacity.HEAVY_CAPACITY_MULTIPLIER)})"
        ]
