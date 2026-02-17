from utc.src.clustering.similarity.dbscan_options import DbscanOptions
from utc.src.constants.options.options import Options
from utc.src.constants.options.general_options import GeneralOptions
from dataclasses import dataclass, asdict, fields
from typing import List, Tuple, Dict, Any, Optional

from utc.src.graph.graph_options import TopkaOptions


# ----------------------- Init -----------------------

@dataclass
class ModeOptions(Options):
    """ Data class for mode initialization options """
    type: str
    interval: Tuple[float, float]
    reserve: int
    routed_eta: bool
    dynamic_cost: bool
    domain: str
    window: int

    def validate_options(self) -> bool:
        return True


@dataclass
class InitOptions(Options):
    """ Data class for routing initialization options """
    # Traffic scenario options
    scenario: str # Name of existing scenario (folder)
    config: str  # Name of config (default same as scenario)
    snapshot: str # Optional path to snapshot file
    # edge_data
    new_scenario: str # Name of newly created scenario
    # Online / Offline mode
    mode: ModeOptions = None

    def validate_options(self) -> bool:
        return True
        # return self.validate_data(asdict(self), "PddlInitOptions")

# ----------------------- Solver -----------------------

@dataclass
class DirOptions(Options):
    """ Data class for routing directory options """
    keep: bool = True
    zip: bool = False

    def validate_options(self) -> bool:
        if not self.keep:
            self.zip = False
        return True


@dataclass
class SolverOptions(Options):
    """ Data class for solver options """
    name: str = "MIP"
    timeout: float = 9
    problems: DirOptions = None
    results: DirOptions = None
    output: DirOptions = None

    def validate_options(self) -> bool:
        return True
        # return self.validate_data(asdict(self), "PddlPlanningOptions")

# ----------------------- Network -----------------------

@dataclass
class NetworkBuilderOptions(Options):
    """ Data class for network builder options """
    regions: List[str] = None
    simplify: bool = True
    cache_size: int = 2000
    topka: TopkaOptions = None
    dbscan: DbscanOptions = None

    def validate_options(self) -> bool:
        return True

# ----------------------- Routing -----------------------

@dataclass
class RoutingOptions(Options):
    """ Data class for vehicle routing """
    general: GeneralOptions = None
    init: InitOptions = None
    solver: SolverOptions = None
    builder: NetworkBuilderOptions = None

    def __init__(self, config: Dict[str, Any]) -> None:
        """
        :param config: Dictionary of configuration options for RoutingOptions
        """
        # TODO check by schema
        if config is None or not config:
            raise AttributeError("Configuration options cannot be None or empty!")
        routing: Optional[dict] = config.get("routing", None)
        if routing is None or not isinstance(routing, dict):
            raise AttributeError("Invalid configuration option 'routing', expected a dictionary!")
        elif not all([v in routing for v in ["init", "solver", "builder"]]):
            raise AttributeError("Invalid configuration option 'routing', missing options!")
        self.general = self.dataclass_from_dict(GeneralOptions, config["general"])
        self.init = self.dataclass_from_dict(InitOptions, routing["init"])
        self.solver = self.dataclass_from_dict(SolverOptions, routing["solver"])
        self.builder = self.dataclass_from_dict(NetworkBuilderOptions, routing["builder"])

    def validate_options(self) -> bool:
        return None not in (self.general, self.init, self.solver, self.builder)


# For testing purposes
if __name__ == '__main__':
    from json import load
    from utc.src.constants.static.file_constants import FilePaths
    file_path: str = FilePaths.CONFIG_FILE.format("mip_online")
    with open(file_path, "r") as json_file:
        data = load(json_file)
        tmp: RoutingOptions = RoutingOptions(data)

