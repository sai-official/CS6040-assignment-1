import heapq
import argparse

#Header parsing
parser = argparse.ArgumentParser(description='Process network simulation files.')
parser.add_argument('-top', '--topology', required=True, help='Path to the topology file')
parser.add_argument('-con', '--connections', required=True, help='Path to the connections file')
parser.add_argument('-rt', '--routing', required=True, help='Path to the routing file')
parser.add_argument('-ft', '--forwarding', required=True, help='Path to the forwarding file')
parser.add_argument('-path', '--pathfile', required=True, help='Path to the path file')
parser.add_argument('-flag', '--flag', required=True, choices=['hop', 'dist'], help='Flag to specify hop-based or distance-based processing')
parser.add_argument('-p', '--pflag', required=True, choices=[0, 1], type=int, help='Flag for processing: 0 for one mode, 1 for another')

args = parser.parse_args()

# Accessing the arguments
print(f"Topology file: {args.topology}")
print(f"Connections file: {args.connections}")
print(f"Routing file: {args.routing}")
print(f"Forwarding file: {args.forwarding}")
print(f"Path file: {args.pathfile}")
print(f"Flag: {args.flag}")
print(f"P Flag: {args.pflag}")

# Data structures to store topology and connection information
class Edge:
    def __init__(self, src, dest, delay, capacity):
        self.src = src
        self.dest = dest
        self.delay = delay
        self.capacity = capacity
        self.weight = 1  # Default weight for hop-based graph

    def set_weight(self, weight_type):
        if weight_type == 'hop':
            self.weight = 1
        elif weight_type == 'delay':
            self.weight = self.delay

class Vertex:
    def __init__(self, node_id):
        self.node_id = node_id
        self.edges = []

    def add_edge(self, edge):
        self.edges.append(edge)

class ConnectionRequest:
    def __init__(self, src, dest, bimin, biave, bimax):
        self.src = src
        self.dest = dest
        self.bimin = bimin
        self.biave = biave
        self.bimax = bimax

class NetworkAdmin:
    def __init__(self):
        self.vcid_counter = 1
        self.vc_table = {}
        self.link_utilization = {}

    def assign_vcid(self, path):
        vcid_list = []
        for i in range(len(path) - 1):
            incoming_link = (path[i], path[i+1])
            outgoing_link = (path[i+1], path[i])

            vcid = self.vc_table.get(incoming_link)
            if vcid is None:
                vcid = self.vcid_counter
                self.vcid_counter += 1

            self.vc_table[incoming_link] = vcid
            self.vc_table[outgoing_link] = vcid
            vcid_list.append(vcid)

        return vcid_list

    def optimistic_admission(self, connection, path, edges):
        b_iequiv = min(connection.bimax, connection.biave + 0.35 * (connection.bimax - connection.bimin))
        for i in range(len(path) - 1):
            link = (path[i], path[i+1])
            if link not in self.link_utilization:
                self.link_utilization[link] = 0
            if b_iequiv > edges[(link[0], link[1])].capacity - self.link_utilization[link]:
                return False
        vcid_list = self.assign_vcid(path)
        for i in range(len(path) - 1):
            link = (path[i], path[i+1])
            self.link_utilization[link] += b_iequiv
        return vcid_list

    def pessimistic_admission(self, connection, path, edges):
        for i in range(len(path) - 1):
            link = (path[i], path[i+1])
            if link not in self.link_utilization:
                self.link_utilization[link] = 0
            if connection.bimax > edges[(link[0], link[1])].capacity - self.link_utilization[link]:
                return False
        vcid_list = self.assign_vcid(path)
        for i in range(len(path) - 1):
            link = (path[i], path[i+1])
            self.link_utilization[link] += connection.bimax
        return vcid_list

def parse_topology(file, weight_type):
    with open(file, 'r') as f:
        node_count, edge_count = map(int, f.readline().split())
        vertices = {i: Vertex(i) for i in range(node_count)}
        edges = {}
        
        for _ in range(edge_count):
            src, dest, delay, capacity = map(int, f.readline().split())
            edge = Edge(src, dest, delay, capacity)
            edge.set_weight(weight_type)  # Set weight based on the type of graph
            
            vertices[src].add_edge(edge)
            vertices[dest].add_edge(edge)
            
            edges[(src, dest)] = edge
            edges[(dest, src)] = edge  # Assuming bidirectional
            
    return vertices, edges
# Function to parse the topology file
# def parse_topology(file):
#     with open(file, 'r') as f:
#         node_count, edge_count = map(int, f.readline().split())
#         vertices = {i: Vertex(i) for i in range(node_count)}
#         edges = {}
#         for _ in range(edge_count):
#             src, dest, delay, capacity = map(int, f.readline().split())
#             edge = Edge(src, dest, delay, capacity)
#             vertices[src].add_edge(edge)
#             vertices[dest].add_edge(edge)
#             edges[(src, dest)] = edge
#             edges[(dest, src)] = edge  # Assuming bidirectional
#     return vertices, edges

# Function to parse the connection file
def parse_connections(file):
    with open(file, 'r') as f:
        connection_count = int(f.readline())
        requests = []
        for _ in range(connection_count):
            src, dest, bimin, biave, bimax = map(int, f.readline().split())
            requests.append(ConnectionRequest(src, dest, bimin, biave, bimax))
    return requests

# Dijkstra's algorithm to find the shortest path
def dijkstra(vertices, start, end):
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
        for edge in vertices[node].edges:
            neighbor = edge.dest if edge.src == node else edge.src
            if neighbor not in visited:
                heapq.heappush(pq, (cost + edge.delay, neighbor, path))
    return (float("inf"), [])

# Function to find two link-disjoint shortest paths
def find_two_shortest_paths(vertices, src, dest):
    path1_cost, path1 = dijkstra(vertices, src, dest)
    if path1_cost == float("inf"):
        return (None, None), (None, None)

    temp_vertices = {k: Vertex(k) for k in vertices}
    for v in temp_vertices:
        temp_vertices[v].edges = [edge for edge in vertices[v].edges if not ((edge.src in path1 and edge.dest in path1) or (edge.dest in path1 and edge.src in path1))]

    path2_cost, path2 = dijkstra(temp_vertices, src, dest)
    return (path1_cost, path1), (path2_cost, path2)

# Function to check if the connection can be admitted
def can_admit_connection(admin, connection, paths, edges, p_value):
    for path_cost, path in paths:
        if path_cost == float("inf"):
            continue

        if p_value == 0:
            admitted = admin.optimistic_admission(connection, path, edges)
        else:
            admitted = admin.pessimistic_admission(connection, path, edges)

        if admitted:
            return True, path, admitted

    return False, [], []

# Main function to process the files and run the virtual circuit switching
def process_virtual_circuit_switching(topology_file, connection_file, p_value,flag_value):
    vertices, edges = parse_topology(topology_file,flag_value)
    requests = parse_connections(connection_file)

    admin = NetworkAdmin()

    routing_table = {node: [] for node in vertices}
    forwarding_table = {node: [] for node in vertices}
    paths_file = []

    for conn_id, request in enumerate(requests):
        paths = find_two_shortest_paths(vertices, request.src, request.dest)
        admitted, path, vcid_list = can_admit_connection(admin, request, paths, edges, p_value)

        if admitted:
            path_cost = sum([edges[(path[i], path[i+1])].delay for i in range(len(path)-1)])
            if flag_value == 'dist':
            	routing_table[request.src].append((request.dest, path, path_cost, path_cost))
            else:
            	routing_table[request.src].append((request.dest, path, path_cost, len(path) - 1))
            for i in range(len(path) - 1):
                forwarding_table[path[i]].append((path[i], path[i+1], vcid_list[i], path[i+1], vcid_list[i]))
            paths_file.append((conn_id, request.src, request.dest, path, vcid_list, len(path) - 1))
        else:
            print(f"Connection {conn_id} from {request.src} to {request.dest} could not be admitted.")

    # Output to routing table file
    with open(args.routing, 'w') as rt:
        for node, routes in routing_table.items():
            rt.write(f"Routing Table for Node {node}:\n")
            for dest, path, delay, cost in routes:
                rt.write(f"Dest: {dest}, Path: {' -> '.join(map(str, path))}, Path Delay: {delay}, Path Cost: {cost}\n")
            rt.write("\n")

    # Output to forwarding table file
    with open(args.forwarding, 'w') as ft:
        for node, forwards in forwarding_table.items():
            ft.write(f"Forwarding Table for Node {node}:\n")
            for src, dest, in_vcid, out_node, out_vcid in forwards:
                ft.write(f"This Router's ID: {src}, Incoming Port: {src}, VC ID: {in_vcid}, Outgoing Port: {dest}, VC ID: {out_vcid}\n")
            ft.write("\n")

    # Output to paths file
    with open(args.pathfile, 'w') as pt:
        pt.write(f"{len(requests)} {len(paths_file)}\n")
        for conn_id, src, dest, path, vcid_list, path_cost in sorted(paths_file):
            pt.write(f"Conn. ID: {conn_id}, Source: {src}, Dest: {dest}, Path: {' -> '.join(map(str, path))}, VC ID List: {', '.join(map(str, vcid_list))}, PathCost: {path_cost}\n")



def main():
    
    # Run the virtual circuit switching process with the input files
    process_virtual_circuit_switching(args.topology, args.connections, p_value=args.pflag,flag_value=args.flag)  # 0 for optimistic, 1 for pessimistic

if __name__ == "__main__":
    main()
    

