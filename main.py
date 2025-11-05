from constants import STARTING_NODE_ID, DESTINATION_NODE_ID
from global_file import get_crow_fly_distance, getNodeName
from a_star import a_star

import cProfile
import pstats
import io

#########################################################################################
#########################################################################################
####################################### Methods #########################################
#########################################################################################
#########################################################################################


def printResults(result: dict):
    """
    Print the given results with:

    - The straight line distance between the start and destination node.

    - The distance found by the algorithm.

    - The path to take, node by node.

    Variables:

    result -- a dictionary holding the results.
    """
    if result:
        # Print straight line distance and the one found. The straight line should be a little shorter.
        print(
            "Distance as the crow flies: %s km"
            % get_crow_fly_distance(STARTING_NODE_ID, DESTINATION_NODE_ID)
        )
        print("Distance: %s km" % result["distance"])
        # Print the path
        i = 0
        for node_id in result["node_path"]:
            print("Node %i: %s" % (i, getNodeName(node_id)))
            i += 1
    else:
        print("path not found")


def test_algorithm_speed(algorithm, algo_name):
    """
    Launches an algorithm given in parameter with cProfile to analyse and print it's
    performances and results.

    Parameters:

    algorithm -- The algorithm function.

    algo_name -- The algorithm name.
    """
    global STARTING_NODE_ID, DESTINATION_NODE_ID
    print("\n" + "-" * 50)
    print("-- %s" % algo_name)
    print("-" * 50)

    profiler = cProfile.Profile()
    result = profiler.runcall(algorithm, STARTING_NODE_ID, DESTINATION_NODE_ID)

    s = io.StringIO()
    ps = pstats.Stats(profiler, stream=s)
    ps.print_stats(1)

    print("\n" + s.getvalue().strip().split("\n")[0] + "\n")
    printResults(result)


#########################################################################################
#########################################################################################
####################################### Testing #########################################
#########################################################################################
#########################################################################################

test_algorithm_speed(a_star, "A *")
