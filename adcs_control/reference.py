import numpy as np

# Define the global variable for previous quaternion
qd_prev = None

def xi_operator(q):
    # Implement the xi_operator function that returns the appropriate matrix for the quaternion q
    # This is a placeholder implementation
    return np.array([[0, -q[0], -q[1], -q[2]],
                     [q[0], 0, q[2], -q[1]],
                     [q[1], -q[2], 0, q[0]],
                     [q[2], q[1], -q[0], 0]])

def calc_wd(qd, dt):
    global qd_prev
    
    if qd_prev is None:
        qd_prev = qd
        return np.zeros((3, qd.shape[0]))
    
    qd_dot = (qd - qd_prev) / dt
    qd_prev = qd
    wd_Array = np.zeros((3, qd.shape[0]))

    for i in range(qd.shape[0]):
        Xi = xi_operator(qd[i, :])
        wd_Array[:, i] = np.linalg.pinv(Xi) @ (2 * qd_dot[i, :])
    
    return wd_Array