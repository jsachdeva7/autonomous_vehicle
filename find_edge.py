import sumolib

net = sumolib.net.readNet(r"C:/Users/Jagat Sachdeva/Documents/autonomous_car/worlds/city_net/sumo.net.xml")
print(net.getBoundary())

target_x, target_y = 51, 47.5267
radius = 30
edges = net.getNeighboringEdges(target_x, target_y, radius)

if len(edges) > 0:
    distancesAndEdges = sorted([(dist, edge) for edge, dist in edges], key=lambda x:x[0])
    dist, closestEdge = distancesAndEdges[0]
    print(f"Number of edges found: {len(edges)}")
    print(f"Closest edge ID: {closestEdge.getID()}")
else:
    print("No edges found")