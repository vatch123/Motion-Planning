# Motion Planning (A* and RRT*)

## Overview
In this assignment, you will implement and compare the performance of search-based and sampling-based motion planning algorithms on several 3-D environments.

### 1. main.py
This file contains examples of how to load and display the environments and how to call a motion planner and plot the planned path. Feel free to modify this file to fit your specific goals for the project. In particular, you should certainly replace Line 104 with a call to a function which checks whether the planned path intersects the boundary or any of the blocks in the environment.

### 2. Planner.py
This file contains an implementation of a baseline planner. The baseline planner gets stuck in complex environments and is not very careful with collision checking. Feel free to modify this file in any way necessary for your own implementation.

### 3. astar.py
This file contains a class defining a node for the A* algorithm as well as an incomplete implementation of A*. Feel free to continue implementing the A* algorithm here or start over with your own approach.

### 4. maps
This folder contains 7 test environments described via a rectangular outer boundary and a list of rectangular obstacles. The start and goal points for each environment are specified in main.py.


The code contains two additional files from the ones mentioned above:
* `collision.py`: This file contains the code for collision check
* `RRT.py`: This file contains the setup code for RRT using the Python Motion Planning Library

There is also a `src` folder which is the modified source code of the Python Motion Planning Library.

## Usage

First create the conda enviroment and activate it using 
```
conda env create --name proj2 --file=environment.yml
conda activate proj2
```

Next the planning code can be run for all environments sequentially by
```
python main.py
```

The above will run the Naive planner, A* and RRT* on all the environments one after the other.

If you only want to run, A* or RRT* respectively on all environments, use the following respectively
```
python main.py -a
```

```
python main.py -r
```


