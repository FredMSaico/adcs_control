import numpy as np
from quaternionFunctions import skew

def cubeSatEquationState(Td, I, U, x):
    """
    Inputs:
        Td: Disturbances (array-like)
        U: Input torque (array-like)
        I: Rigid body inertia tensor (2D array)
        x: State vector [q; w] (array-like)
            q: Attitude quaternion [q1, q2, q3, q4]
            w: Angular rates [wx, wy, wz]
    Returns:
        x_dot: Derivative of the state vector [q_dot; w_dot]
    """
    # Read inputs
    q = np.array(x[:4])
    w = np.array(x[4:])
    
    # Kinematics and Dynamic Equations
    # Quaternions Kinematics
    Xi = np.array([
        [-q[1, 0], -q[2, 0], -q[3, 0]],
        [ q[0, 0], -q[3, 0],  q[2, 0]],
        [ q[3, 0],  q[0, 0], -q[1, 0]],
        [-q[2, 0],  q[1, 0],  q[0, 0]]
    ])
    q_dot = 0.5 * Xi @ w
    
    # Dynamics Equations
    w_dot = np.linalg.inv(I) @ (Td + U - skew(w) @ (I @ w))
    
    # x_dot Vector
    x_dot = np.concatenate((q_dot, w_dot))
    
    return x_dot