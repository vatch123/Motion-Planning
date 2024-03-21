import numpy as np

from src.rrt.rrt_star_bid_h import RRTStarBidirectionalHeuristic
from src.search_space.search_space import SearchSpace


class RRT:
    def __init__(self, environment) -> None:
        self.boundary = environment["boundary"]
        self.blocks = environment["blocks"]
    
    def plan(self, start, goal):

        print("Running Bidirectional RRT star...")

        boundary = self.boundary
        blocks = self.blocks

        # Define the search space
        X_dims = np.vstack((boundary[0,:3], boundary[0,3:6])).T
        obstacles = blocks[:, :6]
        X = SearchSpace(X_dims, obstacles)
        
        # Define the edges and the steps
        Q = np.array([0.5])
        r = 0.05

        # Define the cutoff criteria
        max_samples = 80000
        rewire_count = 100
        prc = 0.01

        # Run the RRT*
        rrt = RRTStarBidirectionalHeuristic(X, Q, tuple(start), tuple(goal), max_samples, r, prc, rewire_count)
        path = rrt.rrt_star_bid_h()

        return np.array(path)
