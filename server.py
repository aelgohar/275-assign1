from heapviz import *
from graph import Graph
from breadth_first_search import *

class CostDistance:
    """
    A class with a method called distance that will return the Euclidean
    between two given vertices.
    """
    def __init__(self, location):
        """
        Creates an instance of the CostDistance class and stores the
        dictionary "location" as a member of this class.
        """
        self.location = location

    def distance(self, e):
        """
        Here e is a pair (u,v) of vertices.
        Returns the Euclidean distance between the two vertices u and v.
        """
        return manhattan_distance(self.location[e[0]], self.location[e[1]])

def manhattan_distance(u, v):
    # manhattan distance between 2 sets
    return ((u[0]-v[0])**2+(u[1]-v[1])**2) ** 0.5

def least_cost_path(graph, start, dest, cost):
    """
    Find and return a least cost path in graph from start vertex to dest vertex.
    Efficiency: If E is the number of edges, the run-time is
    O( E log(E) ).
    Args:
    graph (Graph): The digraph defining the edges between the
    vertices.
    start: The vertex where the path starts. It is assumed
    that start is a vertex of graph.
    dest:  The vertex where the path ends. It is assumed
    that dest is a vertex of graph.
    cost:  A class with a method called "distance" that takes
    as input an edge (a pair of vertices) and returns the cost
    of the edge. For more details, see the CostDistance class
    description below.
    Returns:
    list: A potentially empty list (if no path can be found) of
    the vertices in the graph. If there was a path, the first
    vertex is always start, the last is always dest in the list.
    Any two consecutive vertices correspond to some
    edge in graph.
    """
    reached = {}
    events = BinaryHeap()
    events.insert((start, start), 0)
    while len(events) > 0:
        pair, time = events.popmin()
        if pair[1] not in reached:
            reached[pair[1]] = pair[0]
            for neighbour in graph.neighbours(pair[1]):
                events.insert((pair[1], neighbour), time + cost.distance((pair[1], neighbour)))

    return(get_path(reached, start, dest))

def load_edmonton_graph(filename):
    """
    Loads the graph of Edmonton from the given file.
    Returns two items
    graph: the instance of the class Graph() corresponding to the
    directed graph from edmonton-roads-2.0.1.txt
    location: a dictionary mapping the identifier of a vertex to
    the pair (lat, lon) of geographic coordinates for that vertex.
    These should be integers measuring the lat/lon in 100000-ths
    of a degree.
    In particular, the return statement in your code should be
    return graph, location
    (or whatever name you use for the variables).
    Note: the vertex identifiers should be converted to integers
    before being added to the graph and the dictionary.
    """
    # initializes variables
    vertices = []
    edges = []
    location = {}
    graph = Graph()
    # opens filename for reading and iterates through each line in the file
    # each line is split by a commana delimiter
    # the type of a line is determine by checking for the start character
    with open(filename, 'r') as myfile:
        for line in myfile:
            lineItems = line.split(',')
            if (lineItems[0] == 'V'):
                graph.add_vertex(int(lineItems[1]))
                location[int(lineItems[1])] = (int(float(lineItems[2])*100000),int(float(lineItems[3])*100000))
            elif (lineItems[0] == 'E'):
                graph.add_edge((int(lineItems[1]), int(lineItems[2])))

    return graph, location

def interaction():
    # load graph and location
    graph, location = load_edmonton_graph("edmonton-roads-2.0.1.txt")
    cost = CostDistance(location)

    # take in the user's input
    request = input().strip().split()

    # valid request
    if request[0] == "R":
        closestStart = float('inf')
        closestEnd = float('inf')
        vertexStart = 0
        vertexEnd = 0
        start = (int(request[1]), int(request[2]))
        end = (int(request[3]), int(request[4][:-4]))

        # finds the closest vertices for start and end
        for key, value in location.items():
            if manhattan_distance(start, value) < closestStart:
                vertexStart = key
                closestStart = manhattan_distance(start, value)
            if manhattan_distance(end, value) < closestEnd:
                vertexEnd = key
                closestEnd = manhattan_distance(end, value)

        # get path from start to end
        path = least_cost_path(graph, vertexStart, vertexEnd, cost)

        # print no path if no path found
        if len(path) == 0:
            print("N 0")
        else:
            # print number of way points
            print("N %d" % len(path))

            # start communicating
            if input() == "A":
                for i in range(len(path)):
                    print("W %d %d" % (location[path[i]][0], location[path[i]][1]))
                    if input() != "A":
                        print("INPUT NOT VALID, BREAKING")
                        break
            print("E")

if __name__ == "__main__":
    interaction()
