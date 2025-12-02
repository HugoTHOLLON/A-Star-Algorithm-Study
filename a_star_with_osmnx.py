import cProfile
import io
import pstats

import os.path
import osmnx
import matplotlib.pyplot as plt
import heapq
from haversine import haversine, Unit  # type: ignore


# Get the name of a city/region/country as written in Nominatim
place_name = "Toulouse, Haute-Garonne, Occitania, Metropolitan France, France"
place_name = "Haute-Garonne, Occitania, Metropolitan France, France"
# place_name = "Occitania, Metropolitan France, France"
# place_name = "Metropolitan France"

# Transform this name in a filepath with the extension graphml
filePath = "./cache_data/" + place_name.replace(", ", "_") + ".graphml"
# If there is a file, load it, it's much faster that downloading the entire graph every time
if os.path.isfile(filePath):
    print("Loading graph from " + filePath)
    G = osmnx.load_graphml(filePath)
    print("Graph loaded")
else:
    # If it's the first time, download the graph from Nominatim using the place name.
    # Download only the roads (network_type="drive").
    print("Download graph of: " + place_name)
    G = osmnx.graph_from_place(place_name, network_type="drive")
    print("Graph downloaded")
    print("Saving the downloaded graph in " + filePath)
    osmnx.save_graphml(G, filePath)
    print("Graph saved")

# Simplify the graph: cleans up nodes by collapsing intermediate nodes (ex: nodes that are not
# intersections, just points used to draw a curved road) into one.
# Graph simplification is already done in osmnx.graph_from_place by default.
try:
    G = osmnx.simplify_graph(G)
except:  # noqa: E722  <-- Remove a lint error
    print("Graph cannot be simplified again.")


# Coordinates of different nodes in France (Toulouse, Haute-Garonne, Occitanie, Paris)
START_COORD = (1.3922743, 43.5699769)  # (lon/lat) N7681108802 Basso-cambo
# END_COORD = (1.4629966, 43.6143038)  # (lon/lat) N305142882 - Jolimont
END_COORD = (0.7247218, 43.1077682)  # (lon/lat) N26691893 - Saint-Gaudens
# END_COORD = (4.4041170, 43.8692150)  # (lon/lat) N495652597 - Courbessac (Nîmes)
# END_COORD = (48.8532677, 2.3478864)  # (lat/lon) N11111197772 - Notre-Dame de Paris

orig_node = osmnx.distance.nearest_nodes(G, X=START_COORD[0], Y=START_COORD[1])
dest_node = osmnx.distance.nearest_nodes(G, X=END_COORD[0], Y=END_COORD[1])

# Precompute node coordinates, x=lon, y=lat
node_positions = {node: (data["x"], data["y"]) for node, data in G.nodes(data=True)}
# Without data=True, G.nodes() returns only the node ID

# Draw the base map
_, ax = osmnx.plot_graph(
    G, show=False, close=False, node_size=0, edge_color="#CCCCCC", bgcolor="white"
)

# Highlight start and goal nodes
ax.scatter(*node_positions[orig_node], c="green", s=80, zorder=5, label="Start")
ax.scatter(*node_positions[dest_node], c="red", s=80, zorder=5, label="Goal")

plt.legend()
# Interactive mode on
plt.ion()
plt.show()

# File where the heuristic output will be saved (used for debug)
heuristic_file = open("a_star_heuristic.txt", "w")


def heuristic(id1, id2):
    """Get the heuristic value between 2 nodes. This value is equal to the straight line
    distance between the 2 nodes.

    Args:
        id1 (int): a node ID.
        id2 (int): a node ID, usually the destination node.

    Returns:
        float: the straight line distance between the 2 given nodes in meters.
    """
    global node_positions, haversine_start_to_goal

    # Reverse the coordinates because harversine uses lat/lon while OSM uses lon/lat
    yNode1, xNode1 = node_positions[id1]
    yNode2, xNode2 = node_positions[id2]

    return min(
        haversine((xNode1, yNode1), (xNode2, yNode2), unit=Unit.METERS),
        haversine_start_to_goal,
    )


def reconstruct_path(came_from, current):
    """Gives the path from the start node to the current node given in parameter.

    Args:
        came_from (dict[int, int]): key -> node_id, value -> node_id of the node preceding the
        key (= which node someone need to pass through to access the key id).
        current (int): the currently chosen node.

    Returns:
        list: the path from the origin node to the chosen node (given in parameter).
        The list starts with the origin node and ends with the chosen node.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


def astar_visual(G, start, goal) -> list | None:
    """Executes the A* algorithm while displaying it in a matplotlib graph.

    Args:
        G (MultiDiGraph): graph containing all the nodes and relationships.
        start (int): ID of the start node.
        goal (int): ID of the goal node.

    Returns:
        list|None: the shortest path found by the algorithm as a list of node
        with the start node at [0] and the end node at [1].
    """
    global heuristic_file, ax

    # heapq of all the discovered nodes sorted by their A* score (also called f score)
    open_set: list = []
    heapq.heappush(open_set, (0, start))

    # key -> node_id,
    # value -> node_id of the node preceding the key
    # (= the node someone needs to pass through to access the key id).
    came_from: dict = {}

    # dict with a node id as key and the cost of the path from the start node to this node as value
    g_score = {node: float("inf") for node in G.nodes}
    g_score[start] = 0

    # dict with a node id as key and the A* score of the node as value
    f_score = {node: float("inf") for node in G.nodes}
    f_score[start] = heuristic(start, goal)

    # set containing already explored nodes. These nodes are removed from the open_set
    explored = set()

    # counter to update matplotlib map
    iteration = 0
    last_path_line = None

    while open_set:
        # get the node with the smallest A* score
        _, current = heapq.heappop(open_set)

        # write the A* score and heuristic of the node for debugging
        heuristic_file.write(
            "A*: %.02f m - h: %.02f m - node: %s\n"
            % (_, heuristic(current, goal), current)
        )

        # Check if the goal is reached
        if current == goal:
            return reconstruct_path(came_from, current)

        # if the current node was already explored, immediatly proceed to the next one
        if current in explored:
            continue

        # Mark as explored
        explored.add(current)

        # Process neighbors
        for neighbor in G.neighbors(current):
            edge_data = G[current][neighbor][0]
            # get path cost using currently chosen node
            tentative_g = g_score[current] + edge_data["length"]
            # if score lower, update g_score, f_score and the open set
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # ---------------------------------------------
        # Visualization update every N iterations
        # ---------------------------------------------
        iteration += 1
        # Only update visuals every N loops
        if iteration % 300 == 0:
            # If there was a previously drawn path, recolor it to magenta
            if last_path_line:
                for line in last_path_line:
                    line.set_color("magenta")
                    line.zorder = 4
                    line.set_linewidth(2.1)

            # Draw the current best path in blue
            if came_from:
                path = reconstruct_path(came_from, current)
                x_vals = [node_positions[n][0] for n in path]
                y_vals = [node_positions[n][1] for n in path]

                # Draw new "current best" path and store the handle
                last_path_line = ax.plot(
                    x_vals, y_vals, c="blue", lw=2, alpha=0.9, zorder=3
                )
            plt.pause(0.00001)

    return None


# Get distance to goal for the heuristic (an heuristic should not be greater than the real distance else it can mess up the pathfinding)
haversine_start_to_goal = float("inf")
haversine_start_to_goal = heuristic(orig_node, dest_node)
# Run A* with cProfile to capture the time it takes for the algorithm to run
profiler = cProfile.Profile()
path = profiler.runcall(astar_visual, G, orig_node, dest_node)
# path = astar_visual(G, orig_node, dest_node)

# Use io.StringIO to catch the profiler's stream and not display it on the terminal
s = io.StringIO()
ps = pstats.Stats(profiler, stream=s)
ps.print_stats(1)
# Print the first line of the profiler with the number of methods called and time taken by the algorithm
print("\n" + s.getvalue().strip().split("\n")[0] + "\n")

# Display the final path
if path:
    x_vals = [node_positions[n][0] for n in path]
    y_vals = [node_positions[n][1] for n in path]
    ax.plot(x_vals, y_vals, c="green", lw=3, zorder=6, label="Final Path")
    plt.legend()
    plt.ioff()
    plt.show()
else:
    print("No path found.")
