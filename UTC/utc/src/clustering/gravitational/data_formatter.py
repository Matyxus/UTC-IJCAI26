from utc.src.constants.static.file_constants import DirPaths, FileExtension
from utc.src.constants.file_system.file_types.xml_file import XmlFile, Element
from utc.src.graph.graph import Graph, RoadNetwork
from typing import Dict


class DataFormatter:
    """
    Class formatting networks and edge dumps files to fit
    representation where edge and junction id's are integers (from 0 to N).
    Provides method to add Congestion Index parameter to edge dump files.
    """
    def __init__(self):
        pass

    # ------------------------ Congestion Index ------------------------

    def compute_congestion_index(self, data_file: str, network: str, save: bool = True) -> bool:
        """
        Congestion index (CI) = (actual travel time - free flow travel time) / free flow travel time,
        formula used here to have CI between <0, 1> is: 1 - (free flow travel time / actual travel time ).

        :param data_file: path to data file (edge dump)
        :param network: network from which to extract the maximal speed on edges
        :param save: true if congestion index should be saved in original data file
        :return: True on success, false otherwise
        :raises FileNotFoundError: if files do not exist
        """
        xml_file: XmlFile = XmlFile(data_file)
        if not xml_file.is_loaded():
            raise FileNotFoundError(f"Edge dump: '{data_file}' does not exist!")
        graph: Graph = Graph(RoadNetwork())
        assert(graph.loader.load_map(network))
        # Compute congestion index
        print("Computing congestion indexes")
        for count, node in enumerate(xml_file.root.findall("interval")):
            # print(f"Computing CI for interval: {count}")
            for edge in node.findall("edge"):
                if "traveltime" in edge.attrib and float(edge.attrib["traveltime"]) > 0:
                    # Free flow travel time
                    ftt: float = graph.road_network.get_edge(edge.attrib["id"]).get_travel_time()
                    # Actual travel time
                    att: float = float(edge.attrib["traveltime"])
                    if att <= ftt:  # In case vehicle were faster or as fast as possible (given max speed limit)
                        edge.attrib["congestionIndex"] = "0"
                    else:
                        edge.attrib["congestionIndex"] = str(round(1 - (ftt / att), 3))
                        assert(0 <= float(edge.attrib["congestionIndex"]) <= 1)
                else:  # No travel time was observed here
                    edge.attrib["congestionIndex"] = "0"
        print("Finished computing congestion indexes")
        return True if not save else xml_file.save(xml_file.file_path)

    def congestion_difference(self, edge_data1: str, edge_data2: str, new_file: str) -> bool:
        """
        Creates new file, with congestion index difference, both files must have it already
        calculated, and must be from the same network with the same number of intervals.

        :param edge_data1: Edge data from which we subtract congestion index
        :param edge_data2: Edge data that are subtracted from first data
        :param new_file: name of new file
        :return: true on success, false otherwise
        """
        # Compute congestion index
        print("Computing congestion index difference")
        xml_file1: XmlFile = XmlFile(edge_data1)
        xml_file2: XmlFile = XmlFile(edge_data2)
        if not xml_file1.is_loaded() or not xml_file2.is_loaded():
            return False
        intervals1: list = list(xml_file1.root.findall("interval"))
        intervals2: list = list(xml_file2.root.findall("interval"))
        if len(intervals1) != len(intervals2):
            print("Error, Intervals of edge data files are not equal !")
            return False

        for index, (interval1, interval2) in enumerate(zip(intervals1, intervals2)):
            edges1: Dict[str, Element] = {edge.attrib["id"]: edge for edge in interval1.findall("edge")}
            edges2: Dict[str, Element] = {edge.attrib["id"]: edge for edge in interval2.findall("edge")}
            for edge_id, edge in edges2.items():
                if edge_id not in edges1:
                    interval1.append(edge)
                else:
                    edges1[edge_id].attrib["congestionIndex"] = str(max(round(
                        float(edges1[edge_id].attrib["congestionIndex"]) - float(edge.attrib["congestionIndex"]), 3), 0)
                    )
            # print(f"Computing CI difference for interval: {index}")
            # for edge1, edge2 in zip(interval1.findall("edge"), interval2.findall("edge")):
            #     assert("congestionIndex" in edge1.attrib and "congestionIndex" in edge2.attrib)
            #     if edge2.attrib["id"] not in edges1:
            #         continue
            #
            #     edge1.attrib["congestionIndex"] = str(
            #         max(round(float(edge1.attrib["congestionIndex"]) - float(edge2.attrib["congestionIndex"]), 3), 0)
            #     )
        print("Finished computing congestion index difference")
        return xml_file1.save(new_file)


if __name__ == "__main__":
    # DataFormatter().compute_congestion_index("map_data.xml", "lust")
    scenario: str = "itsc_25200_32400_comb_allowed_central_side_20"
    scenario1: str = "itsc_25200_32400_comb_allowed_20_pddl"
    scenario2: str = "itsc_25200_32400"
    # DataFormatter().congestion_difference(
    #     DirPaths.SCENARIO_STATISTICS.format(scenario1) + f"/{scenario1}_edgeData.out.xml",
    #     DirPaths.SCENARIO_STATISTICS.format(scenario2) + f"/{scenario2}_duo_edgeData.out.xml",
    #     "diff_edgeData.out.xml"
    # )
    DataFormatter().compute_congestion_index(
        DirPaths.SCENARIO_STATISTICS.format(scenario) + f"/{scenario}_edgeData.out.xml",
        "DCC"
    )

