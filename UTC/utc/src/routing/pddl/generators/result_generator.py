from utc.src.constants.static import FilePaths, FileExtension
from utc.src.constants.static.pddl_constants import SOLVERS
from utc.src.constants.file_system.directory_types.scenario_dir import MyDirectory, ScenarioDir
from utc.src.constants.file_system.my_file import MyFile
from utc.src.routing.pddl.base.pddl_problem import PddlProblem
from utc.src.routing.pddl.base.pddl_result import PddlResult
from utc.src.routing.routing_options import SolverOptions
from utc.src.utils.task_manager import TaskManager
import glob
from typing import Optional, List


class ResultGenerator:
    """
    Class handling the generation of pddl result files
    """
    def __init__(self, options: Optional[SolverOptions] = None):
        self.options: Optional[SolverOptions] = options

    def generate_results(
            self, problems: List[PddlProblem], domain: str, planner: str,
            scenario_dir: ScenarioDir, timeout: float = 27.0, processes: int = 1
        ) -> List[Optional[PddlResult]]:
        """

        :param problems: list of pddl Problem files
        :param domain: name of pddl domain
        :param planner: name of planner
        :param scenario_dir: directory of scenario where results will be saved
        :param processes: number of planner calls to run in parallel
        :param timeout: time limit of seconds planner can work
        :return: List of PddlResults, empty if error occurred
        """
        print(f"---" * 15)
        print(f"Planning {len(problems)} pddl problems, domain: {domain}, planner: {planner}")
        print(f"Processes: {processes}, timeout: {timeout}")
        print(f"ETA: ~{(len(problems) // processes) * timeout} seconds.")
        results: List[Optional[PddlResult]] = []
        # Checks
        if not problems:
            print("Received empty list of pddl problems!")
            return results
        elif scenario_dir is None or not scenario_dir.is_loaded():
            print("Scenario directory is invalid!")
            return results
        # Decide between multi and single process approach
        out_dir: MyDirectory = scenario_dir.create_sub_dir("out")
        size_before: int = len(scenario_dir.get_results())
        if processes > 1:  # Multi
            print(f"Starting multi-process queue with: {processes} processes")
            # Create
            task_manager: TaskManager = TaskManager(processes)
            for index, problem in enumerate(problems):
                task_manager.tasks.append((self.generate_result, tuple([
                    scenario_dir.problems.format_file(problem.name + FileExtension.PDDL), domain, planner,
                    scenario_dir.results, timeout, out_dir.create_sub_dir(f"out{index}").dir_path
                ])))
            results = task_manager.start()
        else:  # Single
            results = [
                self.generate_result(
                    scenario_dir.problems.format_file(problem.name + FileExtension.PDDL),
                    domain, planner, scenario_dir.results, timeout, out_dir.dir_path
                )
                for problem in problems
            ]
        # Delete temporary directories for planner output (if options is true)
        MyDirectory.delete_directory(out_dir.dir_path, recursive=True)
        print(f"Finished, generated: {len(scenario_dir.get_results()) - size_before} PDDL result files")
        return results

    def generate_result(
            self, problem_file: str, domain: str, planner: str,
            out_dir: MyDirectory, timeout: float = 27.0,
            working_dir: Optional[str] = None
        ) -> Optional[PddlResult]:
        """
        :param problem_file: path to pddl problem file
        :param domain: name of pddl domain
        :param planner: name of planner
        :param out_dir: directory where result files will be saved
        :param timeout: time limit of seconds planner can work
        :param working_dir: current working directory (where planner stores intermediate results)
        :return: True on success, false otherwise
        """
        # ----- Checks -----
        if timeout < 1:
            print(f"Timeout has to be at least 1 second, got: {timeout}!")
            return None
        elif not MyFile.file_exists(problem_file):
            return None
        elif not MyFile.file_exists(FilePaths.PDDL_DOMAIN.format(domain)):
            return None
        elif not SOLVERS.get_solver(planner):
            return None
        elif not MyFile.get_file_name(problem_file).startswith("problem"):
            print(f"Problem file names has to contain 'problem', got: {MyFile.get_file_name(problem_file)} !")
            return None
        elif out_dir is None or not out_dir.is_loaded():
            print("Received invalid output directory for pddl result files")
            return None
        # Call planner
        result_name: str = MyFile.get_file_name(problem_file).replace("problem", "result")
        result_path: str = out_dir.format_file(result_name) + FileExtension.PDDL
        planner_call: str = SOLVERS.get_solver(planner).format(
            FilePaths.PDDL_DOMAIN.format(domain),
            problem_file,
            result_path,
            timeout
        )
        # Launching WSL from Windows
        if planner.lower() == "mercury2":
            planner_call = planner_call.replace("\\", "/").replace("C:", "/mnt/c")
        # Decide if program has internal timeout, or process needs to be killed
        if planner_call.endswith(str(timeout)):
            success, _ = TaskManager.call_shell_block(planner_call, message=False, cwd=working_dir)
        else:
            success, _ = TaskManager.call_shell(planner_call, timeout=timeout, message=False, cwd=working_dir)
        if not success:
            return None
        # Find the generated files (if they exist)
        files: List[str] = glob.glob(out_dir.format_file(result_name + ".*"))
        if not files:
            return None
        result: PddlResult = PddlResult(result_name, files)
        result.info.timeout = timeout
        result.info.plans = len(files)
        return result

