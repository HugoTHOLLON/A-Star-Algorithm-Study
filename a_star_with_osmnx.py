import cProfile
import io
import pstats

import osmnx
import matplotlib.pyplot as plt
import heapq
from haversine import haversine, Unit  # type: ignore


# Download a graph of the city
place_name = "Toulouse, Haute-Garonne, Occitania, Metropolitan France, France"
place_name = "Haute-Garonne, Occitania, Metropolitan France, France"
# place_name = "Occitania, Metropolitan France, France"
# place_name = "Metropolitan France"
G = osmnx.graph_from_place(place_name, network_type="drive")

# Simplify the graph
try:
    G = osmnx.simplify_graph(G)
except:  # noqa: E722
    print("Graph cannot be simplified again.")


START_COORD = (1.3922743, 43.5699769)  # (lon/lat) N7681108802 Basso-cambo
# END_COORD = (1.4629966, 43.6143038)  # (lon/lat) N305142882 - Jolimont
END_COORD = (0.7247218, 43.1077682)  # (lon/lat) N26691893 - Saint-Gaudens
# END_COORD = (4.4041170, 43.8692150)  # (lon/lat) N495652597 - Courbessac (Nîmes)
# END_COORD = (48.8532677, 2.3478864)  # (lat/lon) N11111197772 - Notre-Dame de Paris
orig_node = osmnx.distance.nearest_nodes(G, X=START_COORD[0], Y=START_COORD[1])
dest_node = osmnx.distance.nearest_nodes(G, X=END_COORD[0], Y=END_COORD[1])

# Precompute node coordinates
node_positions = {
    node: (data["x"], data["y"]) for node, data in G.nodes(data=True)
}  # (lon/lat)

# 2. Draw base map
fig, ax = osmnx.plot_graph(
    G, show=False, close=False, node_size=0, edge_color="#CCCCCC", bgcolor="white"
)

# Highlight start and goal
ax.scatter(*node_positions[orig_node], c="green", s=80, zorder=5, label="Start")
ax.scatter(*node_positions[dest_node], c="red", s=80, zorder=5, label="Goal")

plt.legend()
plt.ion()  # Interactive mode on
plt.show()

# to see heuristic output
heuristic_file = open("a_star_heuristic.txt", "w")

# --- A* Algorithm with Visualization ---


def heuristic(u, v):
    global node_positions

    # Reverse the coordinates as harversine uses lat/lon while OSM uses lon/lat
    yu, xu = node_positions[u]
    yv, xv = node_positions[v]

    return haversine((xu, yu), (xv, yv), unit=Unit.METERS)


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


def astar_visual(G, start, goal):
    global heuristic_file

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}

    g_score = {node: float("inf") for node in G.nodes}
    g_score[start] = 0

    f_score = {node: float("inf") for node in G.nodes}
    f_score[start] = heuristic(start, goal)

    explored = set()

    iteration = 0  # counter to update matplotlib map

    while open_set:
        _, current = heapq.heappop(open_set)

        heuristic_file.write(
            "f: %.02f m - %.02f m - node: %s\n" % (_, heuristic(current, goal), current)
        )

        # Mark as explored
        explored.add(current)

        # Goal reached?
        if current == goal:
            return reconstruct_path(came_from, current)

        # Process neighbors
        for neighbor in G.neighbors(current):
            edge_data = G[current][neighbor][0]
            tentative_g = g_score[current] + edge_data["length"]

            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # ---------------------------------------------
        # Visualization update every 10 iterations
        # ---------------------------------------------
        iteration += 1
        if iteration % 50000 == 0:  # Only update visuals every 10 loops
            # Draw explored nodes
            for node in list(explored)[-50:]:  # draw recent ones for speed
                x, y = node_positions[node]
                ax.scatter(x, y, c="gold", s=10, alpha=0.5, zorder=3)

            # Draw current best reconstructed path
            if came_from:
                path = reconstruct_path(came_from, current)
                x_vals = [node_positions[n][0] for n in path]
                y_vals = [node_positions[n][1] for n in path]
                ax.plot(x_vals, y_vals, c="magenta", lw=2, alpha=0.7, zorder=4)

            plt.pause(0.001)

    return None


# Run A* and visualize
profiler = cProfile.Profile()
path = profiler.runcall(astar_visual, G, orig_node, dest_node)
# path = astar_visual(G, orig_node, dest_node)

s = io.StringIO()
ps = pstats.Stats(profiler, stream=s)
ps.print_stats(1)

print("\n" + s.getvalue().strip().split("\n")[0] + "\n")

# Final path highlight
if path:
    x_vals = [node_positions[n][0] for n in path]
    y_vals = [node_positions[n][1] for n in path]
    ax.plot(x_vals, y_vals, c="blue", lw=3, zorder=5, label="Final Path")
    plt.legend()
    plt.ioff()
    plt.show()
else:
    print("No path found.")
