from typing import Dict


class AssigmentStats:
    """
    Statistics about dynamic route assigment (per solver) to vehicles.
    """
    def __init__(self):
        self.total: int = 0 # How many assignments with DUO/DSO were done
        self.successful: int = 0 # How many succeeded in total
        self.equal: int = 0 # How many assignments matched the original (Or DUO when DSO is overwriting)
        self.used: int = 0 # How many vehicles used this approach assignments (and are different)


class VehicleStats:
    """
    Statistics about vehicles for a specific routing episode (traffic window)
    """
    def __init__(self):
        self.departed: int = 0
        self.considered: int = 0
        self.scheduled: int = 0
        self.routed: int = 0
        self.rescheduled: int = 0 # How many vehicles were re-scheduled
        self.missed: int = 0 # How many vehicles were not routed due to incorrect 'eta'

    def __add__(self, other: 'VehicleStats') -> 'VehicleStats':
        """
        :param other: Class of vehicle statistics
        :return: Self with incremented attributes
        """
        if not isinstance(other, VehicleStats):
            raise TypeError(f"Cannot add {type(other)} into VehicleStats")
        variables: Dict[str, int] = vars(self)
        for key, value in vars(other).items():
            variables[key] += value
        return self
