import numpy as np
import modern_robotics as mr
from motionplanner import motionplanner
from NextState import NextState
from FeedbackControl import FeedbackControl

def finaltest():

    traj = motionplanner()
    config = np.zeros(12)
    Xerr_int = np.zeros(6)

    Kp = 5 * np.eye(6)
    Ki = 0.5 * np.eye(6)
    dt = 0.01

    Tb0 = np.array([
        [1,0,0,0.1662],
        [0,1,0,0],
        [0,0,1,0.0026],
        [0,0,0,1]
    ])

    M0e = np.array([
        [1,0,0,0.033],
        [0,1,0,0],
        [0,0,1,0.6546],
        [0,0,0,1]
    ])

    Blist = np.array([
        [0,0,1,0,0.033,0],
        [0,-1,0,-0.5076,0,0],
        [0,-1,0,-0.3526,0,0],
        [0,-1,0,-0.2176,0,0],
        [0,0,1,0,0,0]
    ]).T

    csv_out = []

    for i in range(len(traj)-1):
        Xd = mr.RpToTrans(traj[i][:9].reshape(3,3), traj[i][9:12])
        Xd_next = mr.RpToTrans(traj[i+1][:9].reshape(3,3), traj[i+1][9:12])

        phi, x, y = config[0:3]
        Tsb = np.array([
            [np.cos(phi), -np.sin(phi), 0, x],
            [np.sin(phi),  np.cos(phi), 0, y],
            [0,0,1,0.0963],
            [0,0,0,1]
        ])

        T0e = mr.FKinBody(M0e, Blist, config[3:8])
        X = Tsb @ Tb0 @ T0e

        V, Xerr, Xerr_int = FeedbackControl(
            X, Xd, Xd_next, Kp, Ki, dt, Xerr_int
        )

        Jarm = mr.JacobianBody(Blist, config[3:8])
        Je = np.hstack((np.zeros((6,4)), Jarm))
        controls = np.linalg.pinv(Je) @ V

        config = NextState(config, controls, dt)
        csv_out.append(np.concatenate((config, [traj[i][-1]])))

    np.savetxt("youbot_trajectory.csv", csv_out, delimiter=",")
    print("CSV generated")

if __name__ == "__main__":
    finaltest()

