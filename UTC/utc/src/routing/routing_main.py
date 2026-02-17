from utc.src.constants.file_system.file_types.json_file import JsonFile, FileExtension
from utc.src.routing.base.controlled_vehicle import ControlledVehicle
from utc.src.routing.routing_options import RoutingOptions, asdict
from utc.src.routing.mode import Mode, Online
from utc.src.utils.vehicle_extractor import VehicleExtractor, VehicleEntry
from xml.etree.ElementTree import Element
from copy import deepcopy
from typing import Optional, List, Dict




class RoutingMain:
    """
    Class handling vehicle routing
    """
    def __init__(self, options: RoutingOptions):
        """
        :param options: of vehicle routing
        """
        assert(options is not None)
        self.options: RoutingOptions = options
        self.mode: Optional[Mode] = None
        self._initialized: bool = False

    def run(self) -> bool:
        """
        Runs the vehicle routing process based on the given options

        :return: True on success, false otherwise
        """
        # Initialize routing mode
        # if self.options.init.mode.type == "offline":
        #     self.mode = Offline(self.options)
        # else:
        self.mode = Online(self.options)
        # Run the routing
        vehicle_mapping: Dict[str, ControlledVehicle] = {vehicle.id: vehicle for vehicle in self.mode.run()}
        if not vehicle_mapping:
            print(f"Error, no vehicles were routed!")
            return False
        # Save the new routes and vehicles to a new scenario
        extractor = VehicleExtractor(self.mode.scenario.vehicles_file, self.mode.scenario.routes_file)
        entry: VehicleEntry = extractor.estimate_arrival_naive((0, float("inf")))
        for vehicle in entry.vehicles.values():
            if vehicle.id in vehicle_mapping:
                # Setup mapping of internal ID to original (for PDDL files reference)
                vehicle.attributes["iid"] = f"v{vehicle_mapping[vehicle.id].internal_id}"
                route: Element = deepcopy(entry.original_routes[vehicle.attributes["route"]])
                route.attrib["edges"] = " ".join(vehicle_mapping[vehicle.id].route.edges)
                vehicle.attributes["route"] = self.mode.new_scenario.routes_file.add_route(route, re_index=True)
            self.mode.new_scenario.vehicles_file.add_vehicle(vehicle.to_xml())
        # Save the newly created routes along with vehicles
        if not self.mode.new_scenario.save(self.mode.scenario.config_file.get_network(), False):
            return False
        # TODO Save additional information files (each subclass of MODE has it as dict)
        # Finally save the options config
        config_path: str = self.mode.new_scenario.scenario_dir.info.format_file("config" + FileExtension.JSON)
        return JsonFile(config_path).save(config_path, asdict(self.options))


if __name__ == '__main__':
    # get_args().get("config")
    config: dict = JsonFile.load_config("comb_test")
    if not config or config is None:
        raise ValueError("Received invalid config!")
    routing_main: RoutingMain = RoutingMain(RoutingOptions(config))
    routing_main.run()
