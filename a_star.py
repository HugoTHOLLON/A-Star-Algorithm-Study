import math

from global_file import (
    NodeWeight,
    get_crow_fly_distance,
    get_neighbors_nodes,
    # printArrayForDebug,
    getNodeName,
)


class AStarScore(NodeWeight):
    """
    Class extending NodeWeight. It is used in the A * algorithm to store 2 new variables:

    - the heuristic value of a node (in this project it is the straight line distance
    between this node and the destination node).

    - the A * score (heuristic value + weight).

    Variables:

    weight -- distance between this node and the starting node.

    parent_node_id -- the id of the preceding node. If None, it means the current node
    is the starting point.

    already_chosen -- boolean checking if this node has already been chosen by dijkstra
    algorithm. This avoid infinite loops where this node is chosen indefinitly.

    heuristic -- the heuristic value of the node.

    a_score -- the A * score of the node. This variable is updated automatically
    when heuristic or weight changes. This variable shouldn't be updated manually.
    """

    a_score: float  # A * cost is equal to weight + heuristic value of node.

    def __init__(
        self,
        heuristic: float,
        weight: float,
        parent_node_id: int | None,
        already_chosen: bool = False,
    ):
        super().__init__(weight, parent_node_id, already_chosen)
        self._weight = weight
        self.heuristic = heuristic

    def __str__(self):
        return (
            "A* Score: %s - Heuristic %s - " % (self.a_score, self._heuristic)
            + super().__str__()
        )

    @property
    def heuristic(self):
        return self._heuristic

    @heuristic.setter
    def heuristic(self, value):
        """Custom setter for heurisitc. Also set a_score when updating heuristic."""
        self._heuristic = value
        self.a_score = self._heuristic + self.weight

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, value):
        """Override setter from NodeWeight: update a_score when weight changes."""
        self._weight = value
        # Recalculate a_score if heuristic already exists
        if hasattr(self, "_heuristic"):
            self.a_score = self._heuristic + self._weight


def get_next_node(a_star_dict: dict[int, AStarScore], previous_node_id: int):
    smallest_id = previous_node_id
    smallest_a_score = math.inf

    for node_id, node_score in a_star_dict.items():
        if not node_score.already_chosen and node_score.a_score < smallest_a_score:
            smallest_a_score = node_score.a_score
            smallest_id = node_id

    a_star_dict[smallest_id].already_chosen = True
    return smallest_id


def a_star(start_id, destination_id, debug=False):
    """Executes the A * algorithm. The distance as crow flies will be used as an heuristic value."""

    a_star_dict: dict[int, AStarScore] = {}

    a_star_dict[start_id] = AStarScore(
        get_crow_fly_distance(start_id, destination_id), 0, None, True
    )

    chosen_node_id = start_id

    while chosen_node_id and chosen_node_id != destination_id:
        neighbors = get_neighbors_nodes(chosen_node_id)

        for index, neighbor in neighbors.iterrows():
            id = neighbor["node_id"]
            heuristic = get_crow_fly_distance(id, destination_id)
            # get the neighbor score (if it was already registered)
            node_score = a_star_dict.get(id)
            # set new distance made of the weight of the currently chosen node and it's
            # distance with the neighbor
            new_distance = (
                neighbor["distance_km"] + a_star_dict.get(chosen_node_id).weight
            )
            # We don't need to use the A* score here because it is composed of the weight
            # and a static value so in any case if the weight is smaller the A* score will
            # be smaller too.

            # if the neighbor's weight isn't in the dict or if it's greater than the new
            # distance, do
            if not node_score or node_score.weight > new_distance:
                # set a new AStarScore for the neighbor
                a_star_dict[id] = AStarScore(
                    heuristic,
                    new_distance,
                    chosen_node_id,
                )
        # end of for each neighbor

        # select the next node (and mark it as already chosen)
        previous_chosen_node = chosen_node_id
        chosen_node_id = get_next_node(a_star_dict, chosen_node_id)

        if debug:
            print(
                "\nChosen node: name: %s - distance: %f - A* score: %f - parent name: %s"
                % (
                    getNodeName(chosen_node_id),
                    a_star_dict[chosen_node_id].weight,
                    a_star_dict[chosen_node_id].a_score,
                    getNodeName(a_star_dict[chosen_node_id].parent_node_id),
                )
            )

        if previous_chosen_node == chosen_node_id:
            return None
    # end of the algorithm, the shortest path has been found
    # format the returned data
    path = {
        "distance": a_star_dict[chosen_node_id].weight,
        "node_path": [chosen_node_id],
    }
    # get all parents using parent_node_id like a linked list starting with the last node.
    parent = a_star_dict[chosen_node_id].parent_node_id
    while parent:
        path["node_path"].append(parent)
        parent = a_star_dict[parent].parent_node_id
    # set the node path like: [start_node, node_1, ..., node_n, end_node]
    path["node_path"].reverse()

    return path
