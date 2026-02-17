from utc.src.constants.options.logging_options import Options, LoggingOptions
from dataclasses import dataclass, asdict


@dataclass
class CpuOptions(Options):
    """ Data class for CPU options """
    threads: int = 8
    processes: int = 4

    def validate_options(self) -> bool:
        return self.validate_data(asdict(self), "CpuOptions")

@dataclass
class GeneralOptions(Options):
    """ Data class for general options (present in all configs) """
    name: str
    config_type: str
    version: str = "0.0.2"
    cpu: CpuOptions = None
    logging: LoggingOptions = None

    def validate_options(self) -> bool:
        return True
        # return self.validate_data(asdict(self), "InfoOptions")
