from shapely.geometry import Polygon, Point, LineString
import networkx as nx
from sklearn.neighbors import KDTree


class Graph:

    def __init__(self, polygons):
        self._polygons = polygons
        self._g = nx.Graph()

    def can_connect(self, n1, n2):

        l = LineString([n1, n2])
        for p in self._polygons:
            if p.crosses(l) and p.height >= min(n1[2], n2[2]):
                return False
        return True

    def create_graph(self, nodes, k):

        tree = KDTree(nodes)
        i = 0
        for n1 in nodes:
            ides = tree.query([n1], k, return_distance=False)[0]
            # print('process {0} nodes, ides nums {1}'.format(i, len(ides)), flush=True)
            i += 1
            for ind in ides:
                n2 = nodes[ind]
                if n1 == n2:
                    continue

                if self.can_connect(n1, n2):
                    self._g.add_edge(n1, n2, weight=1)

    @property
    def graph(self):
        return self._g

    @property
    def edges(self):
        return self._g.edges
