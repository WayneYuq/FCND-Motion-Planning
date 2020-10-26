from shapely.geometry import Polygon, Point, LineString
import networkx as nx
from sklearn.neighbors import KDTree
import numpy as np


class Graph:

    def __init__(self, polygons, polygon_center_tree, max_poly_xy):
        self._polygons = polygons
        self._polygon_center_tree = polygon_center_tree
        self._max_poly_xy = max_poly_xy
        self._g = nx.Graph()
        self._has_false = False

    def can_connect(self, n1, n2, depth=0):
        """Using something like binary search to reduce time cost."""
        # l = LineString([n1, n2])
        # for p in self._polygons:
        #     if p.crosses(l) and p.height >= min(n1[2], n2[2]):
        #         return False
        # return True
        if depth == 0:
            self._has_false = False

        if self._has_false:
            return False

        center = ((n1[0] + n2[0]) / 2, (n1[1] + n2[1]) / 2, (n1[2] + n2[2]) / 2)
        idxs = list(self._polygon_center_tree.query_radius(np.array([center[0], center[1]]).reshape(1, -1),
                                                           r=self._max_poly_xy)[0])

        if len(idxs) > 0:
            for ind in idxs:
                p = self._polygons[int(ind)]
                if p.contains(center) and p.height >= center[2]:
                    self._has_false = True
                    return False

        if depth >= 5:
            return True

        depth += 1
        return self.can_connect(n1, center, depth) and self.can_connect(center, n2, depth)


    def create_graph(self, nodes, k):

        tree = KDTree(nodes)
        i = 0
        for n1 in nodes:
            ides = tree.query([n1], k, return_distance=False)[0]
            print('process {0} nodes, ides nums {1}'.format(i, len(ides)), flush=True)
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
