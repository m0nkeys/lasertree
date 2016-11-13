from __future__ import division
import numpy as np
import matplotlib.pyplot as plt


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Tree(object):
    def __init__(self, id, diameter, position_x, position_y):
        self.id = id
        self.diameter = diameter
        self.position = np.array([position_x, position_y])
        self.intersect = False

    def get_tree_line(self, laser_vector):  # calculate tree line perpendicular to laser vector
        perpendicular_vector = np.array([-1 * laser_vector[1], laser_vector[0]])
        start = self.position - self.diameter / 200. * perpendicular_vector
        end = self.position + self.diameter / 200. * perpendicular_vector

        return start, end


class Laser(object):
    def __init__(self, position_x, position_y, angular_step):
        self.position = np.array([position_x, position_y])
        self.vector = np.array([0, 0])
        self.angular_step = angular_step
        self.angle = 0  # starts at angle 0 deg
        self.calculate_beam_vector()  # richtungsvektor

    def update_angle(self):
        self.angle += self.angular_step
        self.calculate_beam_vector()  # update vector

    def calculate_beam_vector(self):
        self.vector = np.array(([np.cos(self.angle * np.pi / 180.), np.sin(self.angle * np.pi / 180.)]))


class Simulator(object):
    def __init__(self, plane_diameter, number_of_trees, min_diameter, max_diameter):
        self.plane_diameter = plane_diameter
        self.number_of_trees = number_of_trees
        self.min_diameter = min_diameter
        self.max_diameter = max_diameter
        self.trees = []
        self.laser = []
        self.nearest_tree = 0
        self.shadowed = []

        self.create_trees(number_of_trees, 'random', min_diameter, max_diameter)
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

    def create_trees(self, n_trees, mode, min_diameter, max_diameter):
        if mode == 'random':
            for tree_number in range(n_trees):
                # diameter in cm
                diameter = np.random.randint(min_diameter, max_diameter + 1)

                # x and y position inside the circular plane
                pos_x = np.random.randint(int((-self.plane_diameter / 2.0)+ diameter/200.0), int((self.plane_diameter / 2. + 1)-diameter/200.0))

                y_boundary = abs(np.sqrt((self.plane_diameter / 2) ** 2 - pos_x ** 2))

                pos_y = np.random.randint(int((-1 * y_boundary)+ diameter/200.0), int((y_boundary + 1)-diameter/200.0))

                self.trees.append(Tree(tree_number, diameter, pos_x, pos_y))

    def draw_plane(self):
        fig = plt.figure(1)
        plt.axis([-self.plane_diameter / 2, self.plane_diameter / 2, -self.plane_diameter / 2, self.plane_diameter / 2])
        ax = fig.add_subplot(1, 1, 1)

        circle = plt.Circle((0, 0), self.plane_diameter / 2, color='r', fill=False)

        # draw laser
        for laser in self.laser:
            plt.plot(laser.position[0], laser.position[1], 'ro')
            laser_out = laser.position + self.plane_diameter / 2 * laser.vector
            plt.plot([laser.position[0], laser_out[0]], [laser.position[1], laser_out[1]], 'r')

        # fig, ax = plt.subplots()  # note we must use plt.subplots, not plt.subplot
        ax.add_artist(circle)
        for tree in self.trees:
            circle_tree = plt.Circle((tree.position[0], tree.position[1]), tree.diameter / 200., color='g', fill=False)
            #start_tree, end_tree = tree.get_tree_line(self.laser.vector)
            #if tree.id == self.nearest_tree:
            #    plt.plot([start_tree[0], end_tree[0]], [start_tree[1], end_tree[1]], 'r')
            #else:
            #    plt.plot([start_tree[0], end_tree[0]], [start_tree[1], end_tree[1]], 'g')
            ax.add_artist(circle_tree)
            # plt.plot(tree.position_x, tree.position_y,'go')
        plt.show()


sim = Simulator(10, 20, 10, 100)
sim.simulate()
sim.draw_plane()
# sim.test_tree_crossing(sim.trees[0])
