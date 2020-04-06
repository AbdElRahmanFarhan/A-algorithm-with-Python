from matplotlib import pyplot as plt
import numpy as np
from matplotlib.path import Path
import matplotlib.patches as patches
import time



def search(map, cost, start, end, max_iter, max_dist):

    # Create start and end node with initized values for g, h and f
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, tuple(end))
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration.
    # From here we will find the lowest cost node to expand next
    possible_actions = []
    # in this list we will put all node those already explored so that we don't explore it again
    visited_list = []

    # Add the start node
    possible_actions.append(start_node)

    # Adding a stop condition. This is to avoid any infinite loop and stop
    # execution after some reasonable number of steps
    outer_iterations = 0
    travelled_distance = 0

    # what squares do we search . serarch movement is left-right-top-bottom
    # (4 movements) from every positon

    move = [[-1, 0],  # go down
            [0, -1],  # go left
            [1, 0],  # go up
            [0, 1]]  # go right

    # find maze has got how many rows and columns
    no_rows, no_columns = np.shape(map)

    # Loop until you find the end

    while len(possible_actions) > 0:

        # Every time any node is referred from possible actions list, counter of limit operation incremented
        outer_iterations += 1

        # Get the current node
        current_node = possible_actions[0]
        current_index = 0
        for index, item in enumerate(possible_actions):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        if outer_iterations > 1:
            dist_from_parent = ((((current_node.position[0] - current_node.parent.position[0]) ** 2) +
                           ((current_node.position[1] - current_node.parent.position[1]) ** 2))) ** -2
            travelled_distance = travelled_distance + dist_from_parent
        # if we hit this point return the path such as it may be no solution or
        # computation cost is too high
        if outer_iterations > max_iter:
            print("too many iterations")
            return return_path(current_node, map), travelled_distance, outer_iterations

        if travelled_distance > max_dist:
            print("max distance exceeded")
            return return_path(current_node, map), travelled_distance, outer_iterations

        # Pop current node out off possible actions list, add to visited list
        possible_actions.pop(current_index)
        visited_list.append(current_node)

        # test if goal is reached or not, if yes then return the path
        if current_node == end_node:
            return return_path(current_node, map), travelled_distance, outer_iterations

        # Generate children from all adjacent squares
        children = []

        for new_position in move:

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range (check if within maze boundary)
            if (node_position[0] > (no_rows - 1) or
                    node_position[0] < 0 or
                    node_position[1] > (no_columns - 1) or
                    node_position[1] < 0):
                continue

            # Make sure walkable terrain
            if map[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the visited list (search entire visited list)
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + cost
            ## Heuristic costs calculated here, this is using eucledian distance
            child.h = (((child.position[0] - end_node.position[0]) ** 2) +
                       ((child.position[1] - end_node.position[1]) ** 2))

            child.f = child.g + child.h

            # Child is already in the possible actions list and g cost is already lower
            if len([i for i in possible_actions if child == i and child.g > i.g]) > 0:
                continue

            # Add the child to the possible actions list
            possible_actions.append(child)

if __name__ == '__main__':

    # program inputs
    #map size
    size_x = 22
    size_y = 16
    x, y = np.meshgrid(np.arange(size_x), np.arange(size_y))  # make a canvas with coordinates
    x, y = x.flatten(), y.flatten()
    empty_map_coords = np.vstack((x, y)).T

    # obstacles
    A_coords = [(3, 1), (11, 1), (11, 3), (3, 3), (3, 1)]
    B_coords = [(3, 5), (5, 5), (6, 8), (4, 10), (2, 7), (3, 5)]
    C_coords = [(7, 5), (9, 5), (8, 8), (7, 5)]
    D_coords = [(11, 5), (14, 2), (16, 4), (11, 5)]
    E_coords = [(17, 3), (17, 2), (18, 1), (19, 2), (19, 3), (18, 4)]
    F_coords = [(10, 12), (10, 7), (13, 10), (12, 12)]
    G_coords = [(15, 6), (18, 6), (18, 12), (15, 12)]
    H_coords = [(19, 11), (21, 5), (21, 11), (20, 12)]

    A = Obstacle(A_coords, size_x, size_y, empty_map_coords)
    B = Obstacle(B_coords, size_x, size_y, empty_map_coords)
    C = Obstacle(C_coords, size_x, size_y, empty_map_coords)
    D = Obstacle(D_coords, size_x, size_y, empty_map_coords)
    E = Obstacle(E_coords, size_x, size_y, empty_map_coords)
    F = Obstacle(F_coords, size_x, size_y, empty_map_coords)
    G = Obstacle(G_coords, size_x, size_y, empty_map_coords)
    H = Obstacle(H_coords, size_x, size_y, empty_map_coords)



    map = A.coordinates + B.coordinates + C.coordinates + D.coordinates + \
          E.coordinates + F.coordinates + G.coordinates + H.coordinates

    start = [7, 13]  # starting position
    end = [20, 3] # ending position
    cost = 1  # cost per movement
    max_dist = 50
    max_iter = (len(map) // 2) ** 10

    start_time = time.time()

    path, travelled_dist, iterations = search(map, cost, start, end, max_iter, max_dist)

    print ("computation time = ", time.time() - start_time)
    print ("travelled distance = ", travelled_dist)
    print ("iterations = ", iterations)
    # plotting
    ax = plt.axes()
    if path is not None:
        for i in range(size_x):
            for j in range(size_y):
                if path[i][j] == -1:
                    ax.plot(i, j, marker="X", color='k', markersize=5)
                elif path[i][j] == 0:
                    pass
                else:
                    ax.plot(i, j, marker="*", color='y', markersize=11)
    else:
        print ("no solution is found")

    ax.plot(start[0],start[1], marker="s", color='g', markersize=12)
    ax.plot(end[0], end[1], marker="8", color='r', markersize=12)
    ax.add_patch(A.patch)
    ax.add_patch(B.patch)
    ax.add_patch(C.patch)
    ax.add_patch(D.patch)
    ax.add_patch(E.patch)
    ax.add_patch(F.patch)
    ax.add_patch(G.patch)
    ax.add_patch(H.patch)
    plt.xticks(np.arange(0, size_x, step=1))
    plt.yticks(np.arange(0, size_y, step=1))
    plt.grid()
    plt.show()
