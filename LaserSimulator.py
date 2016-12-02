from __future__ import division
import numpy as np


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Tree(object):
    def __init__(self, id, bhd, azimuth, distance):
        self.id = id
        self.bhd = bhd
        self.azimuth = azimuth
        self.distance = distance
        self.intersect = False

    def get_tree_line(self, laser_vector):  # calculate tree line perpendicular to laser vector
        perpendicular_vector = np.array([-1 * laser_vector[1], laser_vector[0]])
        start = self.position - self.diameter / 200. * perpendicular_vector
        end = self.position + self.diameter / 200. * perpendicular_vector

        return start, end


class Laser(object):
    def __init__(self, azimuth, distance, angular_step):
        self.azimuth = azimuth
        self.distance = distance
        self.angular_step = angular_step
        self.angle = 0  # starts at angle 0 deg
        self.calculate_beam_vector()  # richtungsvektor
        self.vector = np.zeros(1)

    def update_angle(self):
        self.angle += self.angular_step
        self.calculate_beam_vector()  # update vector

    def calculate_beam_vector(self):
        self.vector = np.array(([np.cos(self.angle * np.pi / 180.), np.sin(self.angle * np.pi / 180.)]))


class Simulator(object):
    def __init__(self, circle_id, tree_id, distance, azimuth, bhd):
        self.plane_diameter = 0
        self.number_of_trees = 0
        self.trees = []
        self.laser = []

        for i in range(len(tree_id)):
            self.trees.append(Tree(tree_id[i], bhd[i], azimuth[i], distance[i]))

        self.create_laser(0, 0, 1)
        self.create_laser(2, 3, 1)

    def simulate(self):
        for laser in self.laser:
            for i in np.arange(0, 360, laser.angular_step):
                self.calculate_shadow(laser)
                laser.update_angle()

            unique_shadows = np.unique(self.shadowed)
            print("ready")
            print("{0} Trees of {1} Trees are shadowed! This is a percentage of {2}%").format(unique_shadows.size,
                                                                                              self.number_of_trees,
                                                                                              100.0 * unique_shadows.size /
                                                                                              self.number_of_trees)

    def calculate_shadow(self, laser):
        distances = []
        tree_id = []

        for tree in self.trees:
            laser_out = laser.position + self.plane_diameter / 2 * laser.vector
            start_tree, end_tree = tree.get_tree_line(laser.vector)
            a = Point(laser.position[0], laser.position[1])
            b = Point(laser_out[0], laser_out[1])
            c = Point(start_tree[0], start_tree[1])
            d = Point(end_tree[0], end_tree[1])

            tree.intersect = self.intersect(a, b, c, d)
            if tree.intersect:
                # print("Tree intersects!")
                # find nearest intersection
                distances.append(np.sqrt((laser.position[0] - tree.position[0]) ** 2 + (
                laser.position[1] - tree.position[1]) ** 2))
                tree_id.append(tree.id)
        if len(distances) > 0:
            idx = np.argmin(distances)
            self.nearest_tree = tree_id[idx]
            shadowed_trees = tree_id
            del (shadowed_trees[idx])
            for shadow in shadowed_trees:
                if len(shadowed_trees) > 0:
                    self.shadowed.append(shadow)

    def create_laser(self, x, y, angle):
        self.laser.append(Laser(x, y, angle))

    def ccw(self, A, B, C):
        return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)

    def intersect(self, A, B, C, D):
        # http://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        return self.ccw(A, C, D) != self.ccw(B, C, D) and self.ccw(A, B, C) != self.ccw(A, B, D)
