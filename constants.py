import pandas as pd

# Constants
NODES = "data_ariege/osm_nodes.csv"
WAYS = "data_ariege/osm_ways.csv.gz"

### Serre test data
STARTING_NODE_ID = 469819297  # Saint-Pierre-de-Rivière
# STARTING_NODE_ID = 1469646001  # Cos
# DESTINATION_NODE_ID = 1792742726  # Las Prados
# DESTINATION_NODE_ID = 1205464576  # Cabane Coumauzil
DESTINATION_NODE_ID = 1467560343  # Cautirac

### Ariege test data
# STARTING_NODE_ID = 27789470  # Rochers de la Mirouge
# DESTINATION_NODE_ID = 3961818121  # Chanteraine

# Global variables holding all of the nodes and ways data in pandas.DataFrame
if NODES.endswith(".gz"):
    nodes = pd.read_csv(NODES, usecols=["id", "name", "lon", "lat"], compression="gzip")
else:
    nodes = pd.read_csv(NODES, usecols=["id", "name", "lon", "lat"])

if WAYS.endswith(".gz"):
    ways = pd.read_csv(
        WAYS, usecols=["node_from", "node_to", "distance_km"], compression="gzip"
    )
else:
    ways = pd.read_csv(WAYS, usecols=["node_from", "node_to", "distance_km"])
