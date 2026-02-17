from utc.src.graph import Route
from typing import Optional, List, Dict, Tuple, FrozenSet, Set


class Cache:
    """
    Class used for holding generated sub-graphs, provides utility methods.
    """
    def __init__(self, max_size: int = 1500):
        """
        :param max_size: maximal number of sub-graphs which can be stored
        """
        self._memory: Dict[Tuple[int, int], FrozenSet[int]] = {} # (incoming_edge, outgoing_edge) -> subgraph
        self.invalid: Set[Tuple[int, int]] = set() # (incoming_edge, outgoing_edge)
        self.size: int = 0
        self.max_size: int = max_size

    def get_mapping(self, in_edge: int, out_edge: int) -> Optional[FrozenSet[int]]:
        """
        :param in_edge: incoming edge (internal ID)
        :param out_edge: outgoing edge (internal ID)
        :return: Set of internal edges ID's forming subgraph, None if it does not exist
        """
        return self._memory.get((in_edge, out_edge), None)

    def has_mapping(self, in_edge: int, out_edge: int) -> bool:
        """
        :param in_edge: incoming edge
        :param out_edge: outgoing edge
        :return: True if mapping exists, False otherwise
        """
        if (in_edge, out_edge) in self.invalid:
            return True
        return (in_edge, out_edge) in self._memory

    def save_mapping(
            self, in_edge: int, out_edge: int,
            routes: List[Route], replace: bool = False
        ) -> Optional[FrozenSet[int]]:
        """
        :param in_edge: incoming edge (internal ID)
        :param out_edge: outgoing edges(internal ID)
        :param routes: subgraph formed by list of routes
        :param replace: if previous mapping should be replaced
        :return: Set of edges id's of routes forming sub-graph, None if mapping is invalid or error occurred
        """
        # Invalid mapping
        if routes is None or not routes:
            self.invalid.add((in_edge, out_edge))
            return None
        elif not replace and self.get_mapping(in_edge, out_edge) is not None:
            print(f"Cannot replace mapping: {in_edge} -> {out_edge}, as replace is set to false!")
            return None
        elif self.size + 1 > self.max_size:
            print(f"Cannot add mapping: {in_edge} -> {out_edge}, size: {self.size} is at maximum !")
            return None
        self.size += 1
        sub_graph: FrozenSet[int] = frozenset([edge_id for route in routes for edge_id in route.get_edge_ids(True)])
        self._memory[(in_edge, out_edge)] = sub_graph
        return sub_graph

    def clear(self) -> None:
        """
        Resets mapping, clears memory

        :return: None
        """
        self._memory.clear()
        self.invalid.clear()
        self.size = 0
