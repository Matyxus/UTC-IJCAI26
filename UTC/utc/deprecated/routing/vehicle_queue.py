from utc.src.routing.planning.sumo_vehicle import SumoVehicle
from typing import Optional, List, Set, Tuple, Dict


class VehicleQueue:
    """
    Class holding vehicles in different queues from running simulation,
    only vehicles which visit any of the regions are considered.
    """
    def __init__(self):
        self.vehicles: Dict[str, SumoVehicle] = {}
        self.running: Set[str] = set()  # Vehicle currently running in simulation
        self.scheduled: Set[str] = set()  # Vehicle currently considered for planning
        self.arrived: Set[str] = set()  # Vehicles which already left the simulation
        self.discarded: Set[str] = set()

    def add_vehicle(self, vehicle: SumoVehicle) -> bool:
        """
        :param vehicle: to be added to Queue
        :return: True on success, False otherwise
        """
        if vehicle.id in self.vehicles:
            print(f"Error, vehicle: {vehicle.id} is already Queue!")
            return False
        # print(f"Adding vehicle: {vehicle.id} to queue, segments: {len(vehicle.segments)}")
        self.vehicles[vehicle.id] = vehicle
        self.running.add(vehicle.id)
        return True

    def set_arrival(self, vehicles: Tuple[str], time: float) -> Tuple[int, int]:
        """
        :param vehicles: Id of vehicles which arrived
        :param time: Current time
        :return: Total number of vehicle which left simulation and amount missed for planning
        """
        # Go over all vehicles which arrived at their destination (left simulation)
        arrived: int = 0
        missed: int = 0
        for vehicle_id in vehicles:
            # Vehicle does not visit any region, was not considered
            if vehicle_id not in self.vehicles:
                continue
            arrived += 1
            assert(vehicle_id not in self.arrived)
            if not (vehicle_id in self.scheduled or vehicle_id in self.running or vehicle_id in self.discarded):
                print(f"Error, unknown vehicle: '{vehicle_id}' !")
                quit()
            self.arrived.add(vehicle_id)
            self.vehicles[vehicle_id].arrival_time = time
            if vehicle_id in self.running:
                print(f"Vehicle: '{vehicle_id}' arrived at destination, but was not scheduled!")
                self.running.remove(vehicle_id)
                missed += 1
            elif vehicle_id in self.scheduled:
                diff: float = round(abs(self.vehicles[vehicle_id].get_expected_arrival() - time), 3)
                print(f"Vehicle: '{vehicle_id}' left simulation while scheduled, diff: {diff}[s] !")
                self.scheduled.remove(vehicle_id)
                missed += 1
            # Else Discarded
        return arrived, missed
