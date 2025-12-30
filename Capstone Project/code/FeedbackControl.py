import numpy as np
import modern_robotics as mr

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_int):

    Xerr = mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(X) @ Xd))
    Xerr_int = Xerr_int + Xerr * dt

    Vd = mr.se3ToVec(
        (1/dt) * mr.MatrixLog6(np.linalg.inv(Xd) @ Xd_next)
    )

    Ad = mr.Adjoint(np.linalg.inv(X) @ Xd)
    V = Ad @ Vd + Kp @ Xerr + Ki @ Xerr_int

    return V, Xerr, Xerr_int

