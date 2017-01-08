import LaserSimulator as sim

import matplotlib.pyplot as plt
import numpy as np
import csv


def read_tree_csv(filename):

    circle_id = []
    tree_id = []
    distance = []
    azimuth = []
    bhd = []

    # open csv file
    with open(filename, 'rb') as csvfile:
        i = 0
        for line in csvfile.readlines():
            if i > 0:
                array = line.split(';')
                circle_id_current = int(array[0])
                break
            i += 1

    data = []
    # open csv file
    with open(filename, 'rb') as csvfile:
        # get number of columns
        i = 0
        for line in csvfile.readlines():
            if i > 0:
                array = line.split(';')
                circle_id = int(array[0])
                if circle_id == circle_id_current:
                    tree_id.append(int(array[1]))
                    distance.append(float(array[2]))
                    azimuth.append(int(array[3]))
                    bhd.append(int(array[4]))
                else:
                    data.append({'circle_id': circle_id_current, 'tree_id': tree_id, 'distance': distance, 'azimuth': azimuth, 'bhd': bhd})
                    circle_id_current = int(array[0])
                    tree_id = []
                    distance = []
                    azimuth = []
                    bhd = []
                    tree_id.append(int(array[1]))
                    distance.append(float(array[2]))
                    azimuth.append(int(array[3]))
                    bhd.append(int(array[4]))
            i += 1
        data.append({'circle_id': circle_id_current, 'tree_id': tree_id, 'distance': distance, 'azimuth': azimuth, 'bhd': bhd})
    return data


def draw_plane():
    fig = plt.figure(1)
    plt.axis([-1250, 1250, -1250, 1250])
    ax = fig.add_subplot(1, 1, 1)

    circle = plt.Circle((0, 0), 1250, color='r', fill=False)

    # draw laser
    for laser in simulator.laser:
        plt.plot(100*laser.distance * np.sin(laser.azimuth*180/np.pi), 100*laser.distance * np.cos(laser.azimuth*180/np.pi), 'ro')
        #laser_out = laser.position + 1250 * laser.vector
        #plt.plot([laser.position[0], laser_out[0]], [laser.position[1], laser_out[1]], 'r')

    # fig, ax = plt.subplots()  # note we must use plt.subplots, not plt.subplot
    ax.add_artist(circle)
    for tree in simulator.trees:
        circle_tree = plt.Circle((100*tree.distance*np.sin(tree.azimuth*180/np.pi), 100*tree.distance*np.cos(tree.azimuth*180/np.pi)), tree.bhd / 2., color='g', fill=False)
        # start_tree, end_tree = tree.get_tree_line(self.laser.vector)
        # if tree.id == self.nearest_tree:
        #    plt.plot([start_tree[0], end_tree[0]], [start_tree[1], end_tree[1]], 'r')
        # else:
        #    plt.plot([start_tree[0], end_tree[0]], [start_tree[1], end_tree[1]], 'g')
        ax.add_artist(circle_tree)
        # plt.plot(tree.position_x, tree.position_y,'go')
    plt.show()


circles = read_tree_csv('Python_Kreise_Probe.csv')

laser = [{'id': 1, 'distance': 0, 'azimuth': 0, 'step': 0.1}, {'id': 2, 'distance': 10, 'azimuth': 0, 'step': 0.1}]

#for circle in circles:
simulator = sim.Simulator(circles[0], laser)
simulator.calc_intersections()
#simulator.simulate()
# simulator.test_tree_crossing(sim.trees[0])

draw_plane()