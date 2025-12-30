import numpy as np

def NextState(config, controls, dt, max_speed=10):
    controls = np.clip(controls, -max_speed, max_speed)

    phi, x, y = config[0:3]
    arm = config[3:8]
    wheel = config[8:12]

    u = controls[0:4]
    thetadot = controls[4:9]

    arm = arm + thetadot * dt
    wheel = wheel + u * dt

    r = 0.0475
    l = 0.47 / 2
    w = 0.3 / 2

    F = (r/4) * np.array([
        [-1/(l+w),  1/(l+w),  1/(l+w), -1/(l+w)],
        [1, 1, 1, 1],
        [-1, 1, -1, 1]
    ])

    Vb = F @ u
    wbz, vbx, vby = Vb

    if abs(wbz) < 1e-5:
        dx = vbx * dt
        dy = vby * dt
    else:
        dx = (vbx*np.sin(wbz*dt) + vby*(1-np.cos(wbz*dt))) / wbz
        dy = (vby*np.sin(wbz*dt) - vbx*(1-np.cos(wbz*dt))) / wbz

    x += np.cos(phi)*dx - np.sin(phi)*dy
    y += np.sin(phi)*dx + np.cos(phi)*dy
    phi += wbz * dt

    return np.concatenate(([phi, x, y], arm, wheel))

