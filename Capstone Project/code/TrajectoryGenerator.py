import numpy as np
import modern_robotics as mr

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_goal,
                        Tce_grasp, Tce_standoff, k):

    dt = 0.01
    traj = []

    def pack(T, g):
        return np.concatenate((T[:3,:3].flatten(), T[:3,3], [g]))

    Tse_standoff1 = Tsc_init @ Tce_standoff
    Tse_grasp = Tsc_init @ Tce_grasp
    Tse_standoff2 = Tsc_goal @ Tce_standoff
    Tse_release = Tsc_goal @ Tce_grasp

    segments = [
        (Tse_init, Tse_standoff1, 3, 0),
        (Tse_standoff1, Tse_grasp, 1, 0),
        (Tse_grasp, Tse_grasp, 0.63, 1),
        (Tse_grasp, Tse_standoff1, 1, 1),
        (Tse_standoff1, Tse_standoff2, 5, 1),
        (Tse_standoff2, Tse_release, 1, 1),
        (Tse_release, Tse_release, 0.63, 0),
        (Tse_release, Tse_standoff2, 1, 0)
    ]

    for Tstart, Tend, Tf, grip in segments:
        if Tf == 0.63:
            for _ in range(63):
                traj.append(pack(Tstart, grip))
        else:
            N = int(Tf / (dt/k))
            Tlist = mr.ScrewTrajectory(Tstart, Tend, Tf, N, 3)
            for T in Tlist:
                traj.append(pack(T, grip))

    return np.array(traj)

