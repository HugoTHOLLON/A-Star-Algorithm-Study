import csv
import gzip
import heapq

from haversine import haversine, Unit  # type: ignore


#########################################################################################
#########################################################################################
####################################### Classes #########################################
#########################################################################################
#########################################################################################


class AStarScore:
    def __init__(
        self,
        node_id,
        heuristic: float,
        weight: float,
        parent_node_id: int | None,
    ):
        self.node_id = node_id
        self.heuristic = heuristic
        self.weight = weight
        self.parent_node_id = parent_node_id

    @property
    def a_star_score(self):
        return self.heuristic + self.weight

    def __lt__(self, other):
        return self.a_star_score < other.a_star_score


class Node:
    a_star_score: AStarScore

    def __init__(self, lat: float, lon: float):
        self.lat = lat
        self.lon = lon


class Way:
    def __init__(self, neighbor_node_id: int, distance_km: float):
        self.neighbor_node_id = neighbor_node_id
        self.distance_km = distance_km


#########################################################################################
#########################################################################################
############################# Constants and global variables ############################
#########################################################################################
#########################################################################################

NODES = "data_ariege/osm_nodes.csv"
WAYS = "data_ariege/osm_ways.csv.gz"

START_ID: int = 1
DESTINATION_ID: int = 1

nodes: dict[int, Node] = {}
ways: dict[int, list[Way]] = {}

# Stores the straight line distance calculated with get_haversine_distance().
# This is faster than recompute the distance every time get_haversine_distance() is called.
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


def get_heuristic(node_id):
    """
    Returns the heuristic value of a given node. This function could be updated later to
    tweak the algorithm.
    """
    global DESTINATION_ID
    return get_haversine_distance(node_id, DESTINATION_ID)


def a_star() -> dict | None:
    """Executes the A * algorithm. The straight line distance will be used as the heuristic value."""
    global nodes, ways, START_ID, DESTINATION_ID

    chosen_nodes: dict[int, AStarScore] = {}
    # This list is a heap queue
    unexplored_nodes: list[AStarScore] = []

    start_score = AStarScore(START_ID, get_heuristic(START_ID), 0, None)
    chosen_nodes[START_ID] = start_score
    nodes[START_ID].a_star_score = start_score

    chosen_node_id: int = START_ID

    while chosen_node_id and chosen_node_id != DESTINATION_ID:
        # get the currently chosen node
        chosen_node = chosen_nodes.get(chosen_node_id)

        # get neighbor nodes of the currenty chosen nodes
        for way in ways.get(chosen_node_id, []):
            new_distance = way.distance_km + chosen_node.weight if chosen_node else 0

            node_score = chosen_nodes.get(way.neighbor_node_id)
            # If the node hasn't been chose or discovered yet
            if not node_score:
                node_score = nodes[way.neighbor_node_id].a_star_score
                # If the node doesn't have a AStarScore yet add one to the node and heapq
                if not node_score:
                    heuristic = get_heuristic(way.neighbor_node_id)
                    neighbor_a_score = AStarScore(
                        way.neighbor_node_id, heuristic, new_distance, chosen_node_id
                    )
                    nodes[way.neighbor_node_id].a_star_score = neighbor_a_score
                    heapq.heappush(unexplored_nodes, neighbor_a_score)

                # If the node already have a AStarScore but the new distance found is shorter,
                # change the weight and parent_node_id to match the new one. There's no need to
                # use the heuristic value here as it won't change.
                elif node_score.weight > new_distance:
                    node_score.weight = new_distance
                    node_score.parent_node_id = chosen_node_id

        # get the next smallest path
        try:
            smallest_score = heapq.heappop(unexplored_nodes)
        except IndexError:
            return None

        chosen_nodes[smallest_score.node_id] = smallest_score
        chosen_node_id = smallest_score.node_id

    # TODO end of the algorithm
    # # end of the algorithm, the shortest path has been found
    # # format the returned data
    # path = {
    #     "distance": a_star_dict[chosen_node_id].weight,
    #     "node_path": [chosen_node_id],
    # }
    # # get all parents using parent_node_id like a linked list starting with the last node.
    # parent = a_star_dict[chosen_node_id].parent_node_id
    # while parent:
    #     path["node_path"].append(parent)
    #     parent = a_star_dict[parent].parent_node_id
    # # set the node path like: [start_node, node_1, ..., node_n, end_node]
    # path["node_path"].reverse()

    # return path
    return {}
