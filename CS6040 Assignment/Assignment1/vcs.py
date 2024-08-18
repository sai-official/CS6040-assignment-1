import heapq

# Data structures to store topology and connection information
class Edge:
    def __init__(self, src, dest, delay, capacity):
        self.src = src
        self.dest = dest
        self.delay = delay
        self.capacity = capacity

class ConnectionRequest:
    def __init__(self, src, dest, bimin, biave, bimax):
        self.src = src
        self.dest = dest
        self.bimin = bimin
        self.biave = biave
        self.bimax = bimax

# Function to parse the topology file
def parse_topology(file):
    with open(file, 'r') as f:
        node_count, edge_count = map(int, f.readline().split())
        edges = []
        for _ in range(edge_count):
            src, dest, delay, capacity = map(int, f.readline().split())
            edges.append(Edge(src, dest, delay, capacity))
    return node_count, edges

# Function to parse the connection file
def parse_connections(file):
    with open(file, 'r') as f:
        connection_count = int(f.readline())
        requests = []
        for _ in range(connection_count):
            src, dest, bimin, biave, bimax = map(int, f.readline().split())
            requests.append(ConnectionRequest(src, dest, bimin, biave, bimax))
    return connection_count, requests

# Dijkstra's algorithm to find the shortest path
def dijkstra(graph, start, end):
    pq = [(0, start, [])]
    visited = set()
    while pq:
        (cost, node, path) = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)
        path = path + [node]
        if node == end:
            return (cost, path)
        for neighbor, weight in graph[node]:
            if neighbor not in visited:
                heapq.heappush(pq, (cost + weight, neighbor, path))
    return (float("inf"), [])

# Function to build a graph from the edges
def build_graph(edges):
    graph = {}
    for edge in edges:
        if edge.src not in graph:
            graph[edge.src] = []
        graph[edge.src].append((edge.dest, edge.delay))
    return graph

# Function to find two link-disjoint shortest paths
def find_two_shortest_paths(graph, src, dest):
    path1_cost, path1 = dijkstra(graph, src, dest)
    if path1_cost == float("inf"):
        return (None, None), (None, None)
    
    # Remove edges in path1 to find a disjoint path
    temp_graph = {k: [(n, w) for (n, w) in v] for k, v in graph.items()}
    for i in range(len(path1) - 1):
        temp_graph[path1[i]] = [(n, w) for (n, w) in temp_graph[path1[i]] if n != path1[i+1]]
    
    path2_cost, path2 = dijkstra(temp_graph, src, dest)
    
    return (path1_cost, path1), (path2_cost, path2)

# Function to check if the connection can be admitted
def can_admit_connection(request, paths, edges):
    b_equiv = min(request.bimax, request.biave + 0.35 * (request.bimax - request.bimin))
    for path_cost, path in paths:
        if path_cost == float("inf"):
            continue
        can_admit = True
        for i in range(len(path) - 1):
            for edge in edges:
                if edge.src == path[i] and edge.dest == path[i+1]:
                    if b_equiv > edge.capacity:
                        can_admit = False
                        break
                    edge.capacity -= b_equiv
            if not can_admit:
                break
        if can_admit:
            return True, path
    return False, []

# Main function to process the files and run the virtual circuit switching
def process_virtual_circuit_switching(topology_file, connection_file):
    node_count, edges = parse_topology(topology_file)
    connection_count, requests = parse_connections(connection_file)
    
    graph = build_graph(edges)
    
    routing_table = []
    forwarding_table = []
    paths_file = []
    
    for conn_id, request in enumerate(requests):
        paths = find_two_shortest_paths(graph, request.src, request.dest)
        admitted, path = can_admit_connection(request, paths, edges)
        if admitted:
            vcid = len(forwarding_table) + 1
            for i in range(len(path) - 1):
                forwarding_table.append((path[i], path[i+1], vcid))
            paths_file.append((conn_id, request.src, request.dest, path, vcid))
            routing_table.append((request.dest, path, sum([edges[i].delay for i in range(len(path) - 1)])))
        else:
            print(f"Connection {conn_id} from {request.src} to {request.dest} could not be admitted.")
    
    # Output to files
    with open('RoutingTable.txt', 'w') as rt:
        for entry in routing_table:
            rt.write(f"{entry[0]} {entry[1]} {entry[2]}\n")
    
    with open('ForwardingTable.txt', 'w') as ft:
        for entry in forwarding_table:
            ft.write(f"{entry[0]} {entry[1]} {entry[2]}\n")
    
    with open('Paths.txt', 'w') as pt:
        for entry in paths_file:
            pt.write(f"{entry[0]} {entry[1]} {entry[2]} {entry[3]} {entry[4]}\n")

# Run the virtual circuit switching process with the input files
process_virtual_circuit_switching('topology.txt', 'connection.txt')
