import numpy as np
from TrajectoryGenerator import TrajectoryGenerator

def motionplanner():

    Tse_init = np.eye(4)

    Tsc_init = np.array([
        [1,0,0,1],
        [0,1,0,0],
        [0,0,1,0.025],
        [0,0,0,1]
    ])

    Tsc_goal = np.array([
        [0,1,0,0],
        [-1,0,0,-1],
        [0,0,1,0.025],
        [0,0,0,1]
    ])

    Tce_grasp = np.array([
        [0,0,1,0],
        [0,1,0,0],
        [-1,0,0,0],
        [0,0,0,1]
    ])

    Tce_standoff = Tce_grasp.copy()
    Tce_standoff[2,3] = 0.1

    return TrajectoryGenerator(
        Tse_init, Tsc_init, Tsc_goal,
        Tce_grasp, Tce_standoff, 1
    )

