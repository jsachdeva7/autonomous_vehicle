"""
@file sumo_network_analysis.py
@brief Script for analyzing a SUMO road network using sumolib.

This script reads a SUMO road network from an XML file, retrieves its boundary,
and finds the closest edge to a given target coordinate within a specified radius.

Dependencies:
- sumolib (part of SUMO tools)

Usage:
- Ensure SUMO and sumolib are installed and configured properly.
- Update the file path to the correct SUMO network XML file.
"""

import sumolib

# Load the SUMO Network from an XML file
NETWORK_PATH = r"C:/Users/Jagat Sachdeva/Documents/autonomous_car/worlds/city_net/sumo.net.xml"
net = sumolib.net.readNet(NETWORK_PATH)

# Print the network boundary (useful for debugging)
print(net.getBoundary())

# Target coordinates for edge lookup
target_x, target_y = 51, 47.5267 # Example target coordinates
radius = 30 # Search radius for neighboring edges

# Find neighboring edges around the target location within the given radius
edges = net.getNeighboringEdges(target_x, target_y, radius)

if len(edges) > 0:
    # Sort edges by distance to the target point
    distancesAndEdges = sorted([(dist, edge) for edge, dist in edges], key=lambda x:x[0])
    
    # Get the closest edge
    dist, closestEdge = distancesAndEdges[0]
    
    print(f"Number of edges found: {len(edges)}")
    print(f"Closest edge ID: {closestEdge.getID()}")
else:
    print("No edges found")