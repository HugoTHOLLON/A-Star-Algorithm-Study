import csv
import gzip

from haversine import haversine, Unit  # type: ignore


#########################################################################################
#########################################################################################
####################################### Classes #########################################
#########################################################################################
#########################################################################################


class Node:
    def __init__(self, lat: float, lon: float):
        self.lat = lat
        self.lon = lon


class Way:
    def __init__(self, neighbor_node: int, distance_km: float):
        self.neighbor_node = neighbor_node
        self.distance_km = distance_km


class AStarScore:
    def __init__(
        self,
        heuristic: float,
        weight: float,
        parent_node_id: int | None,
        already_chosen: bool = False,
    ):
        self.weight = weight
        self.parent_node_id = parent_node_id
        self.already_chosen = already_chosen
        self.heuristic = heuristic

    @property
    def a_star_score(self):
        return self.heuristic + self.weight


#########################################################################################
#########################################################################################
############################# Constants and global variables ############################
#########################################################################################
#########################################################################################

NODES = "data_ariege/osm_nodes.csv"
WAYS = "data_ariege/osm_ways.csv.gz"

nodes: dict[int, Node] = {}
ways: dict[int, list[Way]] = {}

# Stores the straight line distance calculated with get_haversine_distance().
# This is faster than recompute the distance every time.
heuristics: dict[tuple[int, int], float] = {}


def fill_nodes(reader: csv.DictReader):
    global nodes
    for row in reader:
        node_id = int(row["id"])
        lon = float(row["lon"])
        lat = float(row["lat"])

        nodes[node_id] = Node(lon, lat)


def fill_ways(reader: csv.DictReader):
    global ways
    for row in reader:
        node_from = int(row["node_from"])
        node_to = int(row["node_to"])
        distance = float(row["distance_km"])

        # Set both nodes as we assume the ways are bidirectional
        ways.setdefault(node_from, []).append(Way(node_to, distance))
        ways.setdefault(node_to, []).append(Way(node_from, distance))


# Read the osm_nodes.csv file and place it's data into a dict
if NODES.endswith(".gz"):
    with gzip.open(NODES, mode="rt", newline="") as f:
        # "rt" = read text mode
        fill_nodes(csv.DictReader(f))
else:
    with open(NODES, newline="") as f:
        fill_nodes(csv.DictReader(f))


# Read the osm_ways.csv file and place it's data into a dict
if WAYS.endswith(".gz"):
    with gzip.open(WAYS, mode="rt", newline="") as f:
        fill_ways(csv.DictReader(f))
else:
    with open(WAYS, newline="") as f:
        fill_ways(csv.DictReader(f))


#########################################################################################
#########################################################################################
######################################## Methods ########################################
#########################################################################################
#########################################################################################


def get_haversine_distance(node1_id, node2_id):
    """
    Get the straight line distance between 2 nodes using their id and haversine formula.\n
    This method can also be used to determine the heuristic value of a node in the A * algorithm.\n
    Returns a distance in kilometers.
    """
    global nodes, heuristics

    # if the distance is already stored
    if heuristics.get((node1_id, node2_id)):
        pass
    # else if the distance is already stored but with the opposite tuple
    elif heuristics.get((node2_id, node1_id)):
        heuristics[(node1_id, node2_id)] = heuristics[(node2_id, node1_id)]
    # else the distance wasn't computed yet
    else:
        # Compute the distance using the haversine formula
        node1 = tuple(nodes[node1_id].lat, nodes[node1_id].lon)
        node2 = tuple(nodes[node2_id].lat, nodes[node2_id].lon)
        heuristics[(node1_id, node2_id)] = haversine(node1, node2, unit=Unit.KILOMETERS)
    # return the distance
    return heuristics.get((node1_id, node2_id))


def a_star(start_id: int, destination_id: int):
    """Executes the A * algorithm. The straight line distance will be used as the heuristic value."""

    unexplored_paths: dict[int, AStarScore] = {}
    chosen_paths: dict[int, AStarScore] = {}

    # TODO check if possible to use a dictionnary heapq

    chosen_node_id: int = start_id
