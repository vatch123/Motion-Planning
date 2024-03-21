
# priority queue for OPEN list
from pqdict import pqdict
import numpy as np
from collision import collision_check

class AStarNode(object):
    def __init__(self, pqkey, coord, hval):
        self.pqkey = pqkey
        self.coord = coord
        self.g = np.inf
        self.h = hval
        self.parent_node = None
        self.parent_action = None
        self.closed = False
    def __lt__(self, other):
        return self.g < other.g     


class AStar(object):

    def __init__(self, environment) -> None:
        self.boundary = environment["boundary"]
        self.blocks = environment["blocks"]

    @staticmethod
    def hueristic(cur, goal):
        return np.linalg.norm(cur - goal)
    
    def plan(self, start, goal, epsilon = 1, resolution = 0.5):
        
        print("Running AStar...")
        boundary = self.boundary
        blocks = self.blocks

        # Initialize the graph and open list
        Graph: tuple[tuple, AStarNode] = {}
        OPEN = pqdict()

        # Get all directions where we can move
        numofdirs = 26
        [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
        dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
        dR = np.delete(dR,13,axis=1)
        dR = dR * resolution

        # Start node
        start_key = tuple(start)
        curr = AStarNode(start_key, start, AStar.hueristic(start, goal))
        curr.g = 0

        # Insert in Graph and Priority Queue
        Graph[start_key] = curr
        OPEN[start_key] = curr.g + epsilon * curr.h

        goal_key = tuple(goal)
        Graph[goal_key] = AStarNode(goal_key, goal, 0)

        count = 0
        while(True):
            curr_key = OPEN.popitem()[0]
            curr = Graph[curr_key]
            curr.closed = True
            
            # The current node is the goal
            if curr_key == goal_key:
                break
            if np.linalg.norm(curr.coord - goal) < resolution:
                Graph[goal_key].parent_node = curr
                break

            for k in range(numofdirs):
                child_coord = curr.coord + dR[:,k]

                # Check if the node is valid
                if( child_coord[0] < boundary[0,0] or child_coord[0] > boundary[0,3] or \
                    child_coord[1] < boundary[0,1] or child_coord[1] > boundary[0,4] or \
                    child_coord[2] < boundary[0,2] or child_coord[2] > boundary[0,5] ):
                    continue

                # Check if that path will collide
                line = np.vstack((curr.coord, child_coord)).T
                if collision_check(line, blocks, num=10):
                    continue

                # Find if the node is already in the Graph
                child_key = tuple(child_coord)
                if child_key in Graph:
                    child = Graph[child_key]
                else:
                    child = AStarNode(child_key, child_coord, AStar.hueristic(child_coord, goal))
                
                # If the child is closed skip it
                if child.closed:
                    continue

                step_dist = np.linalg.norm(child_coord - curr.coord)

                # Update the child's distance
                if curr.g + step_dist < child.g:
                    child.g = curr.g + step_dist
                    child.parent_node = curr
                    child.parent_action = k
                    Graph[child_key] = child
                    OPEN[child_key] = child.g + epsilon * child.h
            count += 1
        
        print(f"Number of nodes expanded: {count}")
        curr_node = Graph[goal_key]
        path_coords = []

        while curr_node:
            path_coords.append(curr_node.coord)
            curr_node = curr_node.parent_node

        path_coords = path_coords[::-1]
        path = np.array(path_coords)

        return path
