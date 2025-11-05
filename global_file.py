import pandas as pd

# To compute distances using lat/lon
from haversine import haversine, Unit  # type: ignore

from constants import nodes, ways


class NodeWeight:
    """
    Class representing the weight of a node (the distance between the node and the starting node).
    This class doesn't store the id of the associated node because it is meant to be used in the following dictionary:
    dict[int, NodeWeight] where the key is the node id.

    Variables:

    weight -- distance between this node and the starting node.

    parent_node_id -- the id of the preceding node. If None, it means the current node is the starting point.

    already_chosen -- boolean checking if this node has already been chosen by dijkstra algorithm. This avoid infinite loops where this node is chosen indefinitly.
    """

    def __init__(
        self,
        weight: float,
        parent_node_id: int | None,
        already_chosen: bool = False,
    ):
        self.weight = weight
        self.parent_node_id = parent_node_id
        self.already_chosen = already_chosen

    def __str__(self):
        return "Weight: %s - Parent node: %s - Already chosen: %s" % (
            self.weight,
            self.parent_node_id,
            self.already_chosen,
        )


heuristics: dict = {}


def get_crow_fly_distance(node1_id, node2_id):
    """
    Get the straight line distance between 2 nodes using their id and haversine formula.

    This method can also be used to determine the heuristic value of a node in the A * algorithm.

    Returns a distance in kilometers.
    """
    global nodes, heuristics
    if heuristics.get((node1_id, node2_id)):
        pass
    else:
        # Calculate the distance using the haversine formula
        node1 = tuple(
            nodes[nodes["id"] == node1_id][["lat", "lon"]].iloc[0]
        )  # (lat, lon)
        node2 = tuple(
            nodes[nodes["id"] == node2_id][["lat", "lon"]].iloc[0]
        )  # (lat, lon)
        heuristics[(node1_id, node2_id)] = haversine(node1, node2, unit=Unit.KILOMETERS)
    return heuristics.get((node1_id, node2_id))


def get_neighbors_nodes(node_id):
    """
    Search for all nodes with a way referencing the node in parameter.

    Returns a pandas.DataFrame with 2 keys: 'node_id' and 'distance_km'.
    """
    global ways
    neighbors = pd.concat(
        [
            ways[ways["node_to"] == node_id][["node_from", "distance_km"]].rename(
                columns={"node_from": "node_id"}
            ),
            ways[ways["node_from"] == node_id][["node_to", "distance_km"]].rename(
                columns={"node_to": "node_id"}
            ),
        ],
        ignore_index=True,
    )

    return neighbors


#########################################################################################
#########################################################################################
################################# Debugging methods #####################################
#########################################################################################
#########################################################################################


def printArrayForDebug(dijkstra_dict: dict[int, NodeWeight]):
    print("-" * 50)
    for node_id, weight in dijkstra_dict.items():
        print(
            "Node: %s - Name: %s - %s"
            % (str(node_id), getNodeName(node_id), str(weight))
        )


def getNodeName(node_id):
    global nodes
    match = nodes[nodes["id"] == node_id]["name"]
    if match.empty:
        return "%i" % node_id if node_id else "None"
    name = match.iloc[0]
    return name if pd.notna(name) and name != "" else "%i" % node_id
