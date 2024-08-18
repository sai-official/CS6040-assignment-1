import sys
import numpy as np
import heapq

# Generate adjacency matrices from topology.txt
def generate_adjacency_matrices(topology_file):
    with open(topology_file, 'r') as file:
        # Read the first line to get the number of nodes and edges
        first_line = file.readline().strip().split()
        N = int(first_line[0])
        E = int(first_line[1])

        # Initialize the adjacency matrices
        hop_matrix = np.zeros((N, N), dtype=int)
        delay_matrix = np.full((N, N), sys.maxsize)

        # Set the diagonal to 0 for delay_matrix
        np.fill_diagonal(delay_matrix, 0)

        # Read each subsequent line to get the edges
        for line in file:
            node1, node2, delay, capacity = map(int, line.strip().split())

            # Since the link is bidirectional, update both directions
            hop_matrix[node1][node2] = 1
            hop_matrix[node2][node1] = 1
            delay_matrix[node1][node2] = delay
            delay_matrix[node2][node1] = delay

    return hop_matrix, delay_matrix


# Class to find the shortest and second shortest paths using Dijkstra's algorithm
class ShortestPathAnd2ndShortestDijkstras:
    def __init__(self):
        self.NO_PARENT = -1
        self.path = []  # Nodes in the shortest path
        self.allDists = []  # List of shortest distances

    # Use Dijkstra’s Shortest Path Algorithm, Time O(n^2), n is number of nodes
    # Auxiliary Space O(n)
    def shortestPath(self, adjacencyMatrix, src, dest):
        n = len(adjacencyMatrix[0])
        shortest = {}
        visited = {}
        parents = {}
        for v in range(n):
            shortest[v] = float('inf')  # Use float for a larger range
            visited[v] = False
        shortest[src] = 0.0  # Use float zero
        parents[src] = self.NO_PARENT
        for i in range(1, n):
            pre = -1
            min_dist = float('inf')
            for v in range(n):
                if not visited[v] and shortest[v] < min_dist:
                    pre = v
                    min_dist = shortest[v]
            if pre == -1:
                return
            visited[pre] = True
            for v in range(n):
                dist = adjacencyMatrix[pre][v]
                if dist > 0:
                    # Check for overflow using a large value threshold
                    if min_dist > sys.float_info.max - dist:
                        continue  # Skip updating if overflow is likely
                    new_dist = min_dist + dist
                    if new_dist < shortest[v]:
                        parents[v] = pre
                        shortest[v] = new_dist
        # Check if a path to the destination was found
        if dest in parents:
            self.allDists.append(shortest[dest])
            self.buildPath(dest, parents)



    # Non-recursive implementation to build the path
    def buildPath(self, i, parents):
        self.path = []
        while i != self.NO_PARENT:
            self.path.insert(0, i)
            i = parents[i]

    # Get 2nd shortest by removing each edge in shortest and compare
    # Time O(n^3), Space O(n)
    def find2ndShortest(self, adjacencyMatrix, src, dest):
        preV = -1
        preS = -1
        preD = -1
        mylist = list(self.path)
        for i in range(0, len(mylist) - 1, 1):
            s = mylist[i]
            d = mylist[i + 1]
            if preV != -1:
                adjacencyMatrix[preS][preD] = preV
                adjacencyMatrix[preD][preS] = preV
            preV = adjacencyMatrix[s][d]
            preS = s
            preD = d
            adjacencyMatrix[s][d] = 0
            adjacencyMatrix[d][s] = 0
            self.shortestPath(adjacencyMatrix, src, dest)



# Class for Network Admission Control and file generation
class NetworkAdmissionControl:
    def __init__(self, hop_matrix, delay_matrix, connections_file):
        self.hop_matrix = hop_matrix
        self.delay_matrix = delay_matrix
        self.connections = []
        self.link_utilization = {}
        self.parse_connections(connections_file)

    def parse_connections(self, connections_file):
        with open(connections_file, 'r') as file:
            R = int(file.readline().strip())
            for line in file:
                src, dest, b_min, b_ave, b_max = map(int, line.strip().split())
                self.connections.append({
                    'src': src,
                    'dest': dest,
                    'b_min': b_min,
                    'b_ave': b_ave,
                    'b_max': b_max
                })

    def find_shortest_paths(self, src, dest):
        sp = ShortestPathAnd2ndShortestDijkstras()
        sp.shortestPath(self.delay_matrix, src, dest)
        first_path = list(sp.path)
        sp.find2ndShortest(self.delay_matrix, src, dest)
        second_path = list(sp.path)
        return first_path, second_path

    def optimistic_admission(self, connection, path):
        b_iequiv = min(connection['b_max'], connection['b_ave'] + 0.35 * (connection['b_max'] - connection['b_min']))
        for i in range(len(path) - 1):
            link = (path[i], path[i + 1])
            if b_iequiv > self.hop_matrix[link[0]][link[1]] - self.link_utilization.get(link, 0):
                return False
        for i in range(len(path) - 1):
            link = (path[i], path[i + 1])
            self.link_utilization[link] = self.link_utilization.get(link, 0) + b_iequiv
        return True

    def pessimistic_admission(self, connection, path):
        for i in range(len(path) - 1):
            link = (path[i], path[i + 1])
            if connection['b_max'] > self.hop_matrix[link[0]][link[1]] - self.link_utilization.get(link, 0):
                return False
        for i in range(len(path) - 1):
            link = (path[i], path[i + 1])
            self.link_utilization[link] = self.link_utilization.get(link, 0) + connection['b_max']
        return True

    def process_requests(self, p_value):
        admitted_connections = []
        for connection in self.connections:
            path1, path2 = self.find_shortest_paths(connection['src'], connection['dest'])

            admitted = False
            if p_value == 0:
                admitted = self.optimistic_admission(connection, path1)
            else:
                admitted = self.pessimistic_admission(connection, path1)

            if not admitted and path2:
                if p_value == 0:
                    admitted = self.optimistic_admission(connection, path2)
                else:
                    admitted = self.pessimistic_admission(connection, path2)

            if admitted:
                admitted_connections.append((connection, path1 if admitted else path2))

        return admitted_connections

    


# Example usage
topology_file = 'topology.txt'
connections_file = 'connections.txt'
routing_table_file = 'routingtable.txt'
forwarding_table_file = 'forwardingtable.txt'
paths_file = 'paths.txt'

hop_matrix, delay_matrix = generate_adjacency_matrices(topology_file)
network = NetworkAdmissionControl(hop_matrix, delay_matrix, connections_file)
#network.write_output_files(routing_table_file, forwarding_table_file, paths_file)

print("Routing table, forwarding table, and paths files have been generated.")
