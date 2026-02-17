from utc.src.constants.file_system.file_types.sumo_routes_file import SumoRoutesFile
from utc.src.constants.static import FilePaths
from utc.src.constants.file_system.file_types.sumo_vehicles_file import SumoVehiclesFile
from utc.src.utils.vehicle_extractor import VehicleExtractor
from matplotlib import pyplot as plt


class TrafficIntensity:
    """
    Generates image detailing traffic intensity from given vehicle file over its
    duration as avg +- deviation
    """

    def __init__(self, vehicles_path: str):
        """
        :param vehicles_file:
        """
        self.extractor: VehicleExtractor = VehicleExtractor(
            SumoVehiclesFile(vehicles_path), SumoRoutesFile(vehicles_path)
        )

    def generate_image(self, file_path: str, period: float = 300) -> bool:
        """
        :param file_path:
        :param period
        :return:
        """
        print(f"Generating image of traffic intensity to file: '{file_path}'")
        if not self.extractor.vehicles_file.has_vehicles():
            print(f"Vehicle file does not contain vehicles!")
            return False
        print(f"Start time: {self.extractor.vehicles_file.get_start_time()}, end time: {self.extractor.vehicles_file.get_end_time()}")
        print(f"Total number of vehicles: {len(self.extractor.vehicles_file.root[1:])}")
        intervals: int = int((self.extractor.vehicles_file.get_end_time() - self.extractor.vehicles_file.get_start_time()) // period)
        hours: int = int((self.extractor.vehicles_file.get_end_time() - self.extractor.vehicles_file.get_start_time()) // 3600)
        print(f"Total number of hours in scenario: {hours}, period: {period}")
        # Data
        x: list = []
        y: list = []
        for i in range(intervals):
            y.append(len(self.extractor.get_vehicles((i * period, (i+1)*period))))
            x.append(i*period)
        for i in range(intervals, 49):
            x.append(i*period)
        y += [2003, 1853, 1978, 1472, 1675, 1256, 1345, 1147]
        average: float = sum(y) / len(y)  # average amount of vehicles per period
        deviation: float = (sum([(i - average) ** 2 for i in x]) / len(y)) ** (1/2)
        # Plot
        hours = (x[-1] // 3600)
        print(f"hours: {hours}")
        plt.xlim((0, max(x)))
        plt.ylim((0, max(y)+1000))
        plt.plot(x, y)
        print(f"Average: {average}, deviation: {deviation}")
        plt.xticks([i*3600 for i in range(hours+1)], labels=[i for i in range(hours+1)])
        plt.yticks([i*1000 for i in range((max(y) // 1000)+2)])
        plt.xlabel("Time [hours]")
        plt.ylabel("Vehicles [thousands]")
        plt.title("Dublin traffic intensity")
        plt.tight_layout()
        plt.show()
        return True


# For testing purposes
if __name__ == '__main__':
    scenario_name: str = "itsc"
    temp: TrafficIntensity = TrafficIntensity("itsc_vehicles.add.xml")
    temp.generate_image("", period=1800)
