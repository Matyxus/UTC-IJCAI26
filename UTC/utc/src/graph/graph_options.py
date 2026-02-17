from utc.src.constants.options.options import Options
from dataclasses import dataclass, asdict


@dataclass
class TopkaOptions(Options):
    """ Data class for TopKA* algorithm options """
    c: float = 1.3
    k: int = 3000

    def validate_options(self) -> bool:
        return self.validate_data(asdict(self), "TopkaOptions")
