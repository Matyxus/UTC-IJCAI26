from utc.src.constants.static import FileExtension
from utc.src.routing.pddl.base.pddl_struct import PddlStruct

class PddlProblem(PddlStruct):
    """
    Class extending PddlStruct by ':domain', 'problem', ':metric',
    acts as holder of converted PDDL objects (road network, vehicles, ...)
    """
    def __init__(self, name: str, domain: str, metric: str = "minimize (total-cost)"):
        """
        :param name: of problem
        :param domain: of the problem
        :param metric: metric defining the problem (cost minimization by default)
        """
        super().__init__()
        self.name: str = name
        self.domain: str = domain
        self.metric: str = metric

    # ------------------------------------ Utils ------------------------------------

    def save(self, file_path: str) -> bool:
        """
        :param file_path: path to file which will be created
        :return: True on success, false otherwise
        """
        # Check
        if not file_path.endswith(FileExtension.PDDL):
            file_path += FileExtension.PDDL
        # Conversion to pdl
        print(f"Creating pddl problem: '{self.name}' in: '{file_path}'")
        self.add_init_state("(= (total-cost) 0)")  # Initial situation current cost is 0
        try:
            with open(file_path, "w") as pddl_problem_file:
                pddl_problem_file.write(str(self))
        except OSError as e:
            print(f"Error: '{e}' while generating pddl problem file: {file_path}!")
            return False
        print(f"Successfully created pddl problem file: {file_path}")
        self.clear()
        return True

    # ------------------------------------ Magic Methods ------------------------------------

    def __str__(self) -> str:
        """
        :return: Pddl problem as string -> https://planning.wiki/ref/pddl/problem
        """
        ret_val: str = "(define\n"
        ret_val += f"(problem {self.name})\n"
        ret_val += f"(:domain {self.domain})\n"
        ret_val += super().__str__()
        ret_val += f"(:metric {self.metric})\n"
        return ret_val + ")"
