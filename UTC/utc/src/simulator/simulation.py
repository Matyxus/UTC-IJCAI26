from utc.src.constants.static import DirPaths, FileExtension, FilePaths
from utc.src.constants.file_system.file_types.sumo_config_file import SumoConfigFile
from utc.src.utils.options.SumoOptions import SumoOptions
import traci
import xml.etree.ElementTree as ET
from typing import Optional, Union, List, Dict


class Simulation:
    """
    Class representing SUMO simulation run by TraCI (traci) library.
    Functions as wrapper around function, provides utility methods.
    Has to be run by using "with" keyword.
    """
    def __init__(self, config: Union[str, SumoConfigFile], options: Dict[str, str] = None, snapshot: str = ""):
        """
        :param config: configuration file or path to one
        :param options: simulation options
        :param snapshot: path to snapshot, default None
        """
        self.config: SumoConfigFile = config if isinstance(config, SumoConfigFile) else SumoConfigFile(config)
        self.sumo_options: SumoOptions = SumoOptions(self.config, options=options)
        self.snapshot: str = snapshot
        self.last_vehicle_depart: float = 0
        self._open: bool = False
        self._step: int = 0

    # ------------------------------------------------- Getters -------------------------------------------------

    def get_vehicles(self) -> Optional[List[ET.Element]]:
        """
        :return: List of vehicles as XML Elements at this step, None if error occurred
        """
        if not self.is_running():
            return None
        vehicles: List[ET.Element] = [
            ET.Element("vehicle", {
                "id": vehicle_id,
                "depart": str(traci.vehicle.getDeparture(vehicle_id) - traci.vehicle.getDepartDelay(vehicle_id)),
                "route": traci.vehicle.getRouteID(vehicle_id),
                "type": "CarDefault",
                "departLane": "best",
                "departPos": "random_free",
                "departSpeed": "max",
                "arrivalPos": "max"
            })
            for vehicle_id in traci.simulation.getDepartedIDList()
        ]
        return vehicles

    def get_routes(self) -> Optional[List[ET.Element]]:
        """
        :return: List of routes as XML Elements at this step (from vehicles), None if error occurred
        """
        if not self.is_running():
            return None
        ret_val: List[ET.Element] = [
            ET.Element("route", {
                "id": traci.vehicle.getRouteID(vehicle_id),
                "edges": " ".join(traci.vehicle.getRoute(vehicle_id)),
            })
            for vehicle_id in traci.simulation.getDepartedIDList()
        ]
        return ret_val

    def get_time(self, use_vehicle_time: bool = False) -> float:
        """
        :param use_vehicle_time: measure time by last vehicle departure (False by default)
        :return: Current time in seconds
        """
        if not traci.isLoaded():
            return -1
        elif use_vehicle_time:
            if traci.simulation.getDepartedNumber() > 0:
                vehicle_id: str = traci.simulation.getDepartedIDList()[-1]
                self.last_vehicle_depart = (
                    traci.vehicle.getDeparture(vehicle_id) - traci.vehicle.getDepartDelay(vehicle_id)
                )
            return self.last_vehicle_depart
        return traci.simulation.getTime()

    def save_snapshot(self, snapshot_path: str) -> bool:
        """
        :param snapshot_path:
        :return:
        """
        if not self.is_running():
            return False
        print(f"Saving simulation state at: '{snapshot_path}'")
        traci.simulation.saveState(snapshot_path)
        return True

    def get_step(self) -> int:
        """
        :return: Current simulation step
        """
        return self._step

    # ------------------------------------------------- Utils -------------------------------------------------

    def initialize(self) -> Optional['Simulation']:
        """
        :return: Self if traci loaded simulation, None if error occurred
        """
        print("Entering simulation, loading ...")
        try:
            traci.start(self.sumo_options.create_command())
            self._open = True
            # Start simulation from given state
            if self.snapshot and SumoConfigFile.file_exists(self.snapshot):
                print(f"Loading simulation from snapshot: {self.snapshot}")
                traci.simulation.loadState(self.snapshot)
        except traci.exceptions.FatalTraCIError as e:
            # Closed by user
            if str(e) == "connection closed by SUMO":
                print("Closed GUI, exiting ....")
            else:
                print(f"Error occurred: {e}")
            self._open = False
            return None
        return self

    def close(self) -> None:
        """
        :return:
        """
        if traci.isLoaded() and self._open:
            print("Exiting simulation...")
            traci.close()
            self._open = False
        return

    def step(self) -> bool:
        """
        Performs step on SUMO simulation.

        :return: True on success, false otherwise
        """
        if not self.is_running():
            return False
        traci.simulation.step()
        self._step += 1
        return True

    # noinspection PyMethodMayBeStatic
    def is_running(self, use_end_time: bool = True) -> bool:
        """
        :param use_end_time: If end time of configuration file should be used
        :return: True if simulation is running, false otherwise
        """
        if not traci.isLoaded():
            print("Simulation is not loaded!")
            return False
        elif not (traci.simulation.getMinExpectedNumber() > 0):
            print("Simulation ended!")
            return False
        elif use_end_time and (traci.simulation.getTime() >= self.config.get_end_time()):
            print(f"Simulation ended at time: {traci.simulation.getTime()}")
            return False
        return True

    # ------------------------------------------------- Magics -------------------------------------------------

    def __del__(self) -> None:
        """
        Closes the running simulation if there is one

        :return: None
        """
        self.close()

    def __enter__(self) -> Optional['Simulation']:
        """
        :return: New Simulation instance, None if error occurred
        """
        return self.initialize()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """
        :param exc_type:
        :param exc_val:
        :param exc_tb:
        :return: None
        """
        self.close()


# For testing purposes
if __name__ == "__main__":
    pass

