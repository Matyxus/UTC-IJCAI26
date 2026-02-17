from utc.src.graph import RoadNetwork
from utc.src.routing.planning.sumo_vehicle import SumoVehicle, SumoRoute, VehicleStats
from utc.src.routing.planning.vehicle_queue import VehicleQueue
from utc.src.simulator.simulation import Simulation, traci
from typing import Optional, List, Set, Tuple, Dict


class Scheduler:
    """
    Class scheduling vehicles from running simulation for online planning.
    """
    def __init__(self, regions: List[RoadNetwork], interval: Tuple[float, float, float]):
        """
        :param regions:
        :param interval:
        """
        self.regions: List[RoadNetwork] = regions
        self.low, self.mid, self.high = interval
        print(self.low, self.mid, self.high)
        assert(self.low < self.mid < self.high)
        self.queue: VehicleQueue = VehicleQueue()
        self.stats: VehicleStats = VehicleStats()
        self.prev_assigned: Set[Tuple[str, SumoRoute]] = set()
        self.reserve: int = 1
        print(f"Initialize scheduler with: {len(regions)} regions, interval: {interval}, reserve: {self.reserve}")

    def step(self, simulation: Simulation) -> bool:
        """
        :param simulation: the current running SUMO simulation
        :return: True on success, False otherwise
        """
        if not simulation.is_running():
            return False
        current_time: float = simulation.get_time()
        # Process vehicles which arrived this time step (i.e. left simulation)
        arrived, missed = self.queue.set_arrival(traci.simulation.getArrivedIDList(), current_time)
        self.stats.arrived += arrived
        self.stats.missed += missed
        # Process vehicles which just entered the network
        for vehicle_id in traci.simulation.getDepartedIDList():
            self.insert_vehicle(vehicle_id, current_time)
        for vehicle_id in self.queue.scheduled:
            segment: SumoRoute = self.queue.vehicles[vehicle_id].get_current_segment()
            current: int = traci.vehicle.getRouteIndex(vehicle_id)
            if current >= segment.first_edge_index:
                segment.arrival = current_time
        return True

    def assign_planned(self, planned_vehicles: List[str], network: RoadNetwork) -> List[str]:
        """
        :param planned_vehicles: vehicles which were planned
        :param network: road network
        :return: List of vehicles which routes can be assigned (by traci)
        """
        for (vehicle_id, segment) in self.prev_assigned:
            if vehicle_id in self.queue.arrived:
                self.stats.correct += 1
                continue
            self.stats.correct += (traci.vehicle.getRouteIndex(vehicle_id) >= segment.first_edge_index)
        if self.prev_assigned:
            print(f"Total: {self.stats.correct}/{len(self.prev_assigned)} assigned planned vehicle arrived to region")
            self.prev_assigned.clear()
        print(f"Assigning planned vehicles, total: {len(planned_vehicles)}")
        ret_val: List[str] = []
        # Check if vehicles can still have their route assigned
        current_time: float = traci.simulation.getTime()
        for vehicle_id in planned_vehicles:
            if vehicle_id not in self.queue.scheduled:
                # assert(vehicle_id in self.queue.arrived)
                self.stats.missed += 1
                continue
            vehicle: SumoVehicle = self.queue.vehicles[vehicle_id]
            segment: SumoRoute = vehicle.get_current_segment()
            assert(segment is not None)
            current: int = traci.vehicle.getRouteIndex(vehicle_id)
            self.queue.scheduled.remove(vehicle_id)
            # Check the vehicle position against the region, reserve and before
            if current >= segment.first_edge_index:
                # diff: float = round(current_time - vehicle.get_expected_arrival(), 3)
                # print(f"Planned vehicle: {vehicle_id} already arrived to region, by: {diff}[s]!")
                self.stats.missed += 1
                if vehicle.switch_segment():
                    self.queue.running.add(vehicle_id)
                else:
                    self.queue.discarded.add(vehicle_id)
            # On reserve  self.reserve > 0 and
            elif current <= (segment.first_edge_index - 1):
                eta: float = self.compute_eta(vehicle_id, network, offset=0)
                # Try to reschedule
                if eta >= self.high:
                    # print(f"Planned vehicle: {vehicle_id} could be re-scheduled, eta: {eta}[s] to region")
                    # print(f"Travel time: {round(traci.edge.getTraveltime(vehicle.route[current]), 3)}")
                    self.stats.rescheduled += 1
                    self.queue.running.add(vehicle_id)
                    segment.flag = True
                    continue
                # TODO check if vehicle can switch lane (without teleporting)
                # print(f"Planned vehicle: {vehicle_id} is on reserve, assigning!")
                ret_val.append(vehicle_id)
                self.stats.planned += 1
            else:  # Before reserve
                # Check if vehicle is in reserve edges
                eta: float = self.compute_eta(vehicle_id, network, self.reserve)
                assign: bool = (eta < self.mid)
                # print(f"Planned vehicle: {vehicle_id} eta to reserve is: {eta}[s], assigning: {assign}")
                if assign:
                    ret_val.append(vehicle_id)
                    self.stats.planned += 1
                else:
                    self.stats.rescheduled += 1
                    self.queue.running.add(vehicle_id)
        # Record assigned vehicles, re-check next time
        # Put vehicles which were assigned back to queue (if possible)
        for vehicle_id in ret_val:
            self.prev_assigned.add((vehicle_id, self.queue.vehicles[vehicle_id].get_current_segment()))
            if self.queue.vehicles[vehicle_id].switch_segment():
                self.queue.running.add(vehicle_id)
            else:
                self.queue.discarded.add(vehicle_id)
        # Process vehicle which may not have been planned
        for vehicle_id in self.queue.scheduled:
            self.stats.missed += 1
            if self.queue.vehicles[vehicle_id].switch_segment():
                self.queue.running.add(vehicle_id)
            else:
                self.queue.discarded.add(vehicle_id)
        # Clear scheduled queue
        self.queue.scheduled.clear()
        return ret_val

    # ------------------------------------------- ETA -------------------------------------------

    def update_travel_time(self, network: RoadNetwork, routed: bool = False) -> None:
        """
        :param network: For which average edge travel time should be updated
        :param routed: True if dynamic routing is used, False otherwise
        :return: None
        """
        print(f"Updating travel time on edges, routed: {routed}")
        # Check if routing is enabled and there is vehicle
        if routed and self.queue.running:
            v_id: str = next(iter(self.queue.running))
            for edge in network.edges.values():
                edge.attributes["travelTime"] = float(traci.vehicle.getParameter(v_id, f"device.rerouting.edge:{edge.id}"))
        else:
            for edge in network.edges.values():
                edge.attributes["travelTime"] = round(traci.edge.getTraveltime(edge.id), 3)
        return

    def compute_etas(self, network: RoadNetwork) -> Optional[List[SumoVehicle]]:
        """
        :param network: On which ETA's should be calculated
        :return: List of vehicles to be scheduled for planning (can be empty)
        """
        print(f"Computing eta for: {len(self.queue.running)} vehicles")
        # Update ETA's of vehicles based on new information
        scheduled: List[SumoVehicle] = []  # List of vehicles scheduled for routing
        discarded: List[str] = []
        for vehicle_id in self.queue.running:
            vehicle: SumoVehicle = self.queue.vehicles[vehicle_id]
            route: SumoRoute = vehicle.get_current_segment()
            assert(route is not None)
            route.eta = self.compute_eta(vehicle_id, network, self.reserve)
            # Unable to compute ETA
            if route.eta == -1:
                discarded.append(vehicle_id)
                continue
            route.eta_calculated = traci.simulation.getTime()
            # Vehicle arrives too soon to region, change segment to next (if possible)
            # TODO after switching calculate the ETA again!
            if route.eta < self.low:
                # print(f"Vehicle: {vehicle_id} eta to region is too soon: {route.eta}[s], switching")
                self.stats.missed += 1
                if not vehicle.switch_segment():
                    # print(f"Unable to switch to next segment of vehicle: {vehicle_id}, discarding!")
                    discarded.append(vehicle_id)
            # If vehicle ETA is within expected interval, consider it for scheduling
            elif self.low <= route.eta <= self.high:
                # print(f"Scheduling vehicle: '{vehicle.id}', eta: {route.eta}[s] to reserve")
                scheduled.append(vehicle)
            # else:  # eta > high -> keep vehicle in queue for next estimation step
            #    print(f"Vehicle: {vehicle_id} has too high ETA: {route.eta}, will be estimated next step")
        # Remove all discarded vehicles
        for vehicle_id in discarded:
            self.stats.missed -= 1  # Discarded contains missed
            self.stats.removed += 1
            self.queue.running.remove(vehicle_id)
            self.queue.discarded.add(vehicle_id)
        # Remove all scheduled vehicles from running
        for vehicle in scheduled:
            self.stats.scheduled += 1
            self.queue.running.remove(vehicle.id)
            self.queue.scheduled.add(vehicle.id)
        print(f"Scheduling {len(scheduled)} vehicles for planning")
        return scheduled

    def compute_eta(self, vehicle_id: str, network: RoadNetwork, offset: int = 1) -> float:
        """
        :param vehicle_id: Id of vehicle for which eta is computed
        :param network: Road network on which the ETA is computed
        :param offset: Offset for eta computing (0 if arrival to region)
        :return: Vehicle's ETA to their current region, -1 if not possible to compute
        """
        vehicle: SumoVehicle = self.queue.vehicles[vehicle_id]
        route: SumoRoute = vehicle.get_current_segment()
        assert(route is not None and vehicle_id not in self.queue.arrived)
        current: int = traci.vehicle.getRouteIndex(vehicle_id)
        assert (current >= 0)  # Vehicle must have departed!
        offset = (0 if route.flag else offset)
        # Calculate reserve index
        index: int = route.first_edge_index - offset
        if index < 0:
            print(f"Unable to compute ETA for vehicle: '{vehicle_id}' before region by: {offset}")
            print("Shifting prediction to be one edge before region")
            index = max((route.first_edge_index - 1), 1)
        # Vehicle is already on or past the entry edge
        while current >= index:
            # print(f"Vehicle: {vehicle_id} is already on/passed the edge: '{vehicle.route[index]}'")
            self.stats.missed += 1
            if not vehicle.switch_segment():
                # print(f"Unable to calculate eta of vehicle: '{vehicle_id}' to: '{vehicle.route[index]}', already on the edge!")
                return -1
            route = vehicle.get_current_segment()
            index = route.first_edge_index - offset
            if index < 0:
                print(f"Unable to compute ETA for vehicle: {vehicle_id} with before region by: {offset}")
                print("Shifting prediction to be one edge before region")
                index = max((route.first_edge_index - 1), 1)
        # Compute eta
        eta: float = network.get_edge(vehicle.route[current]).attributes["travelTime"]
        lane_id = traci.vehicle.getLaneID(vehicle_id)
        # Compute fraction of already traveled length vs total
        fraction: float = 1
        # If we are on internal lane, then the position along the current lane is 0
        if str(lane_id).startswith(vehicle.route[current]):
            fraction -= (traci.vehicle.getLanePosition(vehicle_id) / traci.lane.getLength(lane_id))
        assert(0 <= fraction <= 1)
        eta *= fraction
        for edge in network.get_edges(vehicle.route[(current+1):index]):
            eta += edge.attributes["travelTime"]
        return round(eta, 3)

    # ------------------------------------------- Vehicle arrival -------------------------------------------

    def insert_vehicle(self, vehicle_id: str, current_time: float) -> None:
        """
        :param vehicle_id: Id of vehicle which just entered simulation
        :param current_time: Time of vehicle departure
        :return: None
        """
        self.stats.departed += 1
        route: Tuple[str] = traci.vehicle.getRoute(vehicle_id)
        route_length: int = len(route)
        assert(traci.vehicle.getRouteIndex(vehicle_id) == 0)
        # Go over route edges, check if any are inside any of the regions
        skip: int = 0
        visited: List[str] = []
        segments: List[SumoRoute] = []
        for index, edge_id in enumerate(route):
            # Skip segments in one of the regions
            if skip > 0:
                skip -= 1
                continue
            # Go over regions
            for reg_id, region in enumerate(self.regions):
                if edge_id not in region.edges:
                    continue
                # Vehicle starts inside the region, such routes cannot be planned
                elif index == 0:
                    # Count how many edges should be skipped to get to the next region
                    for region_edge in route[index:]:
                        if region_edge not in region.edges:
                            skip -= 1
                            break
                        skip += 1
                    break
                # Find all edges vehicle travels on inside the region, with entry and exit edges
                visited.append(route[index - 1])  # Entry edge
                assert(route[index - 1] not in region.edges)
                visited.append(edge_id)  # First edges
                for region_edge in route[(index+1):]:
                    # Also append the last edge
                    if region_edge not in region.edges:
                        visited.append(region_edge)  # Exit edge
                        break
                    skip += 1
                    visited.append(region_edge)  # Edges in region
                assert(len(set(visited)) == len(visited))
                # Create SumoRoute (segment)
                segments.append(SumoRoute(reg_id, edge_id, index, visited))
                visited = []
                break
        # Vehicle was considered for scheduling
        if len(segments) != 0:
            self.stats.added += 1
            route_id: str = traci.vehicle.getRouteID(vehicle_id)
            vehicle: SumoVehicle = SumoVehicle({"id": vehicle_id, "route": route_id}, route, current_time, segments)
            self.queue.add_vehicle(vehicle)
        return
