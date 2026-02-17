from utc.src.routing.base.controlled_vehicle import ControlledVehicle
from typing import Optional, List, Set, Tuple, Dict


class VehicleQueue:
    """
    Class holding vehicles in different queues from running simulation
    """
    def __init__(self):
        self.vehicles: Dict[str, ControlledVehicle] = {}
        self.running: Set[str] = set()  # Vehicle currently running in simulation
        self.arrived: Set[str] = set()  # Vehicles which already left the simulation
        self.removed: Set[str] = set() # Vehicles which were removed from routing for some reason

    def add_vehicle(self, vehicle: ControlledVehicle) -> bool:
        """
        :param vehicle: to be added to Queue
        :return: True on success, False otherwise
        """
        if vehicle.id in self.vehicles:
            print(f"Error, vehicle: {vehicle.id} is already Queue!")
            return False
        self.vehicles[vehicle.id] = vehicle
        self.running.add(vehicle.id)
        return True

    def set_arrival(self, vehicles: List[str]) -> Tuple[int, int]:
        """
        :param vehicles: Identifiers of vehicles which arrived
        :return: Total number of vehicle which left simulation and amount missed for planning
        """
        # Go over all vehicles which arrived at their destination (left simulation)
        # print(f"Vehicles: {vehicles} that reached their destination")
        arrived: int = 0
        missed: int = 0
        for vehicle_id in vehicles:
            # Vehicle was not considered for routing
            if vehicle_id not in self.vehicles:
                continue
            arrived += 1
            assert(vehicle_id not in self.arrived)
            if not (vehicle_id in self.running or vehicle_id in self.removed):
                print(f"Error, unknown vehicle: '{vehicle_id}' !")
                quit()
            self.arrived.add(vehicle_id)
            if vehicle_id in self.running:
                missed += (self.vehicles[vehicle_id].route.get_current_segment().routed_by == "")
                self.running.remove(vehicle_id)
            # Else removed
        return arrived, missed
