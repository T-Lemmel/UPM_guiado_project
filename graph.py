import networkx as nx
import matplotlib.pyplot as plt
import math

def create_graph():
   
   # list of nodes
    points = [
        (0, 0),
        (0, 1.5),
        (3, 1.5),
        (3, 5),
        (0, 5),
        (-2, 5),
        (-2, 1.5),
        (0, 8),
        (0, 7),
        (-2.5, 7),
        (-2.5, 9),
        (1, 11.5),
        (1.5, 11.5)
    ]

    # list of edges
    edges = [
        (points[0], points[1]), (points[1], points[2]), (points[2], points[3]), (points[1], points[4]), (points[4], points[5]), (points[5], points[6]), 
        (points[4], points[7]), (points[4], points[8]), (points[7], points[8]), (points[8], points[9]), (points[9], points[10]), (points[10], points[11]), (points[11], points[12])
    ]
    
    # Create a graph
    G = nx.Graph()

    # Add nodes to the graph
    for point in points:
        G.add_node(point)

    # Add edges to the graph
    for edge in edges:
        point1, point2 = edge
        weight = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        G.add_edge(point1, point2, weight=weight)
    
    return G, points

def add_node_and_find_path(G, points, new_point, connect_point):
    " This functions is used to add a new node to the graph and find the path from the new node to the origin (0, 0), in our case to backtack to the origin"

    # Add the new node
    G.add_node(new_point)
    points.append(new_point)

    # Add an edge between the new node and the specified node
    G.add_edge(new_point, connect_point, weight=math.sqrt((new_point[0] - connect_point[0]) ** 2 + (new_point[1] - connect_point[1]) ** 2))

    # Get the path from the new node to the origin (0, 0)
    path = nx.astar_path(G, new_point, (0, 0))

    # Draw the graph and the path
    pos = {point: point for point in points}
    nx.draw(G, pos, with_labels=True, node_size=500, node_color='skyblue', font_size=10, font_weight='bold', font_color='black')
    nx.draw_networkx_nodes(G, pos, nodelist=path, node_color='red', node_size=500)
    nx.draw_networkx_edges(G, pos, edgelist=[(path[i], path[i+1]) for i in range(len(path)-1)], edge_color='red', width=2)
    plt.show()

    return path

if __name__ == "__main__":
    G, points = create_graph()
    path = add_node_and_find_path(G, points, (0,0), (0,0))
    print(path)

