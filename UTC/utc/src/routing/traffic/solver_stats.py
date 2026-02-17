
class SolverStats:
    """
    Statistics related so vehicle routing specific to each solver
    """
    class Vehicles:
        """
        Statistics about vehicles for solvers
        """
        def __init__(self):
            self.received: int = 0
            self.received_unique: int = 0
            self.considered: int = 0
            self.considered_unique: int = 0
            self.generated: int = 0
            self.generated_unique: int = 0

    class Routes:
        """
        Statistics about routes (segments) for solvers
        """
        def __init__(self):
            self.received: int = 0
            self.unique: int = 0
            self.considered: int = 0
            self.considered_unique: int = 0
            self.generated: int = 0
            self.generated_unique: int = 0


    def __init__(self):
        self.vehicles: SolverStats.Vehicles = SolverStats.Vehicles()
        self.routes: SolverStats.Routes = SolverStats.Routes()
        self.total_time: float = 0.
        self.avg_time: float = 0. # Avg. time per traffic window
        self.min_time: float = 0.
        self.max_time: float = 0.

