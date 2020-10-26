import numpy as np
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point
from udacidrone.frame_utils import global_to_local


class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]

    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)


def extract_polygons(data):

    polygons = []
    centers = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        top = north - d_north
        bottom = north + d_north
        left = east - d_east
        right = east + d_east
        corners = [(top, left), (top, right), (bottom, right), (bottom, left)]
        height = alt + d_alt

        p = Poly(corners, height)
        polygons.append(p)
        centers.append((north, east))

    return polygons, centers

class Sampler:

    def __init__(self, data, global_home):
        self._polygons, centers = extract_polygons(data)
        self._global_home = global_home

        self._xmin = np.min(data[:, 0] - data[:, 3])
        self._xmax = np.max(data[:, 0] + data[:, 3])

        self._ymin = np.min(data[:, 1] - data[:, 4])
        self._ymax = np.max(data[:, 1] + data[:, 4])

        self._zmin = 0
        self._zmax = 20
        self._max_poly_xy = 2 * np.max((data[:, 3], data[:, 4]))
        centers = np.array(centers)
        self._tree = KDTree(centers, metric='euclidean')

    def sample(self, num_samples):

        xvals = np.random.uniform(self._xmin, self._xmax, num_samples)
        yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)

        samples = list(zip(xvals, yvals, zvals))

        pts = []
        for s in samples:
            in_collision = False
            idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1),
                                                r=self._max_poly_xy)[0])
            if len(idxs) > 0:
                for ind in idxs:
                    p = self._polygons[int(ind)]
                    if p.contains(s) and p.height >= s[2]:
                        in_collision = True
            if not in_collision:
                pts.append(s)

        return pts

    def sample_goal(self, nodes, lat, lon):

        zvals = np.random.uniform(self._zmin, self._zmax)
        lp = global_to_local((lon, lat, 0), self._global_home)  # (north, east, down)
        goal = (lp[0], lp[1], zvals)

        in_collision = False
        idxs = list(self._tree.query_radius(np.array([goal[0], goal[1]]).reshape(1, -1),
                                            r=self._max_poly_xy)[0])
        if len(idxs) > 0:
            for ind in idxs:
                p = self._polygons[int(ind)]
                if p.contains([goal[0], goal[1], goal[2]]) and p.height >= goal[2]:
                    in_collision = True
        if not in_collision:
            return goal

        # If the lat and lon given by parameters is collide with obstacles,
        # Set it to the closest point
        nodes = np.array(nodes)
        min_dist_idx = np.linalg.norm(np.array(goal) - nodes, axis=1).argmin()

        return nodes[min_dist_idx][0], nodes[min_dist_idx][1], zvals

    @property
    def polygons(self):
        return self._polygons

    @property
    def max_poly_xy(self):
        return self._max_poly_xy

    @property
    def polygon_center_tree(self):
        return self._tree
