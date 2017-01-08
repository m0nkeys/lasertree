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

        # self.intersect = False

    def get_tree_line(self, laser_vector):  # calculate tree line perpendicular to laser vector
        perpendicular_vector = np.array([-1 * laser_vector[1], laser_vector[0]])
        start = self.position - self.diameter / 200. * perpendicular_vector
        end = self.position + self.diameter / 200. * perpendicular_vector

        return start, end

    def get_tree_coordinates(self):
        return Point(self.distance * np.cos(self.azimuth), self.distance * np.sin(self.azimuth))


class Laser(object):
    def __init__(self, id, azimuth, distance, angular_step):
        self.id = id
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

    def get_laser_coordinates(self):
        return Point(self.distance * np.cos(self.azimuth), self.distance * np.sin(self.azimuth))


class Simulator(object):
    def __init__(self, circle, laser):

        self.circle_id = circle['circle_id']
        self.distance = circle['distance']

        self.plane_diameter = 0
        self.number_of_trees = 0
        self.trees = []
        self.laser = []
        self.intersections = []

        for i in range(len(circle['tree_id'])):
            self.trees.append(Tree(circle['tree_id'][i], circle['bhd'][i], circle['azimuth'][i], circle['distance'][i]))

        for l in laser:
            self.laser.append(Laser(l['id'], l['distance'], l['azimuth'], l['step']))

    def calc_intersections(self):
        for laser in self.laser:
            l0 = laser.get_laser_coordinates()
            l1 = Point(300 * np.cos(laser.angle), 300 * np.sin(laser.angle))
            a = (l1.y - l0.y) / (l1.x - l0.x)
            b = 1
            c = (l0.x * (l1.y - l0.y) / (l1.x - l0.x)) + l0.y

            for tree in self.trees:
                t0 = tree.get_tree_coordinates()
                d = c - a * t0.x - b * t0.y
                r_squared = tree.bhd ** 2

                x1 = t0.x + (a * d + b * np.sqrt(r_squared * (a ** 2 + b ** 2) - d ** 2)) / (a ** 2 + b ** 2)
                x2 = t0.x + (a * d - b * np.sqrt(r_squared * (a ** 2 + b ** 2) - d ** 2)) / (a ** 2 + b ** 2)

                y1 = t0.y + (b * d - a * np.sqrt(r_squared * (a ** 2 + b ** 2) - d ** 2)) / (a ** 2 + b ** 2)
                y2 = t0.y + (b * d + a * np.sqrt(r_squared * (a ** 2 + b ** 2) - d ** 2)) / (a ** 2 + b ** 2)

                intersections = [Point(x1, y1), Point(x2, y2)]

                #if r_squared*(a**2 + b**2) == d**2: # ein schnittpunkt

                #elif r_squared*(a**2 + b**2) > d**2: # zwei schnittpunkte

                #else: # kein Schnittpunkt

            return intersections

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

    def ccw(self, A, B, C):
        return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)

    def intersect(self, A, B, C, D):
        # http://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        return self.ccw(A, C, D) != self.ccw(B, C, D) and self.ccw(A, B, C) != self.ccw(A, B, D)
