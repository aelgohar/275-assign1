from heapviz import *
from graph import Graph
from breadth_first_search import breadth_first_search

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
        self.distance = int()
        self.location = location

    def distance(self, e):
        """
        Here e is a pair (u,v) of vertices.
        Returns the Euclidean distance between the two vertices u and v.
        """
        return sqrt((v[0]-u[1])**2+(v[1]-u[1])**2)


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
        if pair[1] != dest:
            reached[pair[1]] = pair[0]
            for neighbour in graph.neighbours(pair[1]):
                events.insert((pair[1], neighbour), time + cost(v, neighbour
    return reached

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

    # opens filename for reading and iterates through each line in the file
    # each line is split by a commana delimiter
    # the type of a line is determine by checking for the start character
    with open(filename, 'r') as myfile:
        for line in myfile:
            lineItems = line.split(',')
            if (lineItems[0] == 'V'):
                vertices.append(lineItems[1])
                location[int(lineItems[1])] = (int(float(lineItems[2])*100000),int(float(lineItems[3])*100000) )
            elif (lineItems[0] == 'E'):
                edges.append((lineItems[1], lineItems[2]))

    # create graph with vertices and edges obtained from graph
    graph = Graph()
    for v in vertices: graph.add_vertex(v)
    for e in edges: graph.add_edge(e)
    for e in edges: graph.add_edge((e[1], e[0]))

    return graph, location

def interaction():
    request = input().strip().split()
    start = [request[1], request[2]]
    end = [request[3], request[4][:-3]]


if __name__ == "__main__":
    # Code for processing route finding requests here
