import networkx as nx
import matplotlib.pyplot as plt

class Edge:
    def __init__(self, from_node, to, weight):
        self.from_node = from_node
        self.to = to
        self.weight = weight

def bellman_ford(edges, vertices, source, distance, previous):
    INF = float('inf')
    distance[source] = 0

    for _ in range(vertices - 1):
        for edge in edges:
            if distance[edge.from_node] != INF and distance[edge.from_node] + edge.weight < distance[edge.to]:
                distance[edge.to] = distance[edge.from_node] + edge.weight
                previous[edge.to] = edge.from_node

    # Check for negative cycles
    for edge in edges:
        if distance[edge.from_node] != INF and distance[edge.from_node] + edge.weight < distance[edge.to]:
            raise RuntimeError("Negative cycle detected")

def reconstruct_shortest_path(source, destination, previous):
    path = []
    current = destination
    while current != source:
        path.append(current)
        current = previous[current]
    path.append(source)
    path.reverse()
    return path

def add_node(edges, vertices, tail, head, weight):
    edges.append(Edge(tail, head, weight))
    vertices = max(vertices, tail, head)
    return vertices

def delete_edge(edges, tail, head):
    new_edges = [edge for edge in edges if (edge.from_node, edge.to) != (tail, head)]
    return new_edges

def main():
    vertices, edges = map(int, input("Enter the number of vertices and edges: ").split())
    edge_list = []

    for i in range(edges):
        tail, head, weight = map(int, input(f"Enter edge {i + 1} tail, head, and weight: ").split())
        edge_list.append(Edge(tail, head, weight))

    source_vertex = 1  # The source vertex (change this as per your requirements)
    distance = [float('inf')] * (vertices + 1)
    previous = [-1] * (vertices + 1)

    try:
        bellman_ford(edge_list, vertices, source_vertex, distance, previous)

        # Print the routing table with table headers
        print("Destination\t\tNext Node\t\tDistance\tNodes in Path")
        for i in range(1, vertices + 1):
            path = reconstruct_shortest_path(source_vertex, i, previous)
            next_node = path[1] if len(path) > 1 else '1'

            print(f"Vertex {i}\t\tVertex {next_node}\t\t{distance[i]}\t\t{' -> '.join(map(str, path))}")

        # Find the shortest of all paths
        shortest_path = float('inf')
        shortest_vertex = -1
        for i in range(1, vertices + 1):
            if i != source_vertex and distance[i] < shortest_path:
                shortest_path = distance[i]
                shortest_vertex = i

        # Display the shortest distance
        print(f"Shortest of all paths from Source Vertex {source_vertex}: {shortest_path}")

        # Route maintenance by addition and deletion of edges
        while True:
            option = int(input("Enter option (1: Add Edge, 2: Delete Edge, 3: Exit): "))
            if option == 1:
                tail, head, weight = map(int, input("Enter the tail, head, and weight of the new edge: ").split())
                edge_list.append(Edge(tail, head, weight))
                vertices = add_node(edge_list, vertices, tail, head, weight)
                distance = [float('inf')] * (vertices + 1)  # Resize the distance list after adding an edge
            elif option == 2:
                tail = int(input("Enter the tail of the edge to be deleted: "))
                head = int(input("Enter the head of the edge to be deleted: "))
                edge_list = delete_edge(edge_list, tail, head)
                # Perform Bellman-Ford again after modification
                bellman_ford(edge_list, vertices, source_vertex, distance, previous)
            elif option == 3:
                break
            else:
                print("Invalid option. Please try again.")

        # Print the updated routing table
        print("Updated Routing Table:")
        print("Destination\t\tNext Node\t\tDistance\tNodes in Path")
        for i in range(1, vertices + 1):
            path = reconstruct_shortest_path(source_vertex, i, previous)
            next_node = path[1] if len(path) > 1 else '1'

            print(f"Vertex {i}\t\tVertex {next_node}\t\t{distance[i]}\t\t{' -> '.join(map(str, path))}")

    except RuntimeError as e:
        print(f"Error: {e}")

    # Create a NetworkX graph
    G = nx.Graph()

    # Add edges to the graph based on your edge_list (assuming edge_list is a list of tuples)
    edge_list = [(edge.from_node, edge.to, edge.weight) for edge in edge_list]
    G.add_weighted_edges_from(edge_list)

    # Define layout for node positions (e.g., using the spring layout)
    pos = nx.spring_layout(G)

    # Draw nodes
    nx.draw_networkx_nodes(G, pos, node_size=300)

    # Draw edges
    nx.draw_networkx_edges(G, pos)

    # Add edge labels (weights)
    edge_labels = {(edge[0], edge[1]): edge[2] for edge in G.edges(data='weight')}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    # Draw labels for nodes
    nx.draw_networkx_labels(G, pos)

    # Display the graph
    plt.axis('off')  # Turn off axis labels
    plt.title("Graph Visualization")
    plt.show()

if __name__ == "__main__":
    main()
