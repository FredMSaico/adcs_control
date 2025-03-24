import numpy as np
from quaternionFunctions import skew, error_quaternio

def boskovic_control(w, wd, dq, delta, k, Umax):
    """
    LS2125204: Brayan Espinoza
    Inputs:
        w:     Satellite angular rate
        wd:    Desired angular rate
        dq:    Error quaternion
        delta: quaternion gain
        k:     Adaptive gain
        Umax:  Saturation torque
    """
    
    dq0 = dq[0]
    dq13 = dq[1:4]
    
    we = w - wd
    
    # Sliding surface
    S = we + k**2 * dq13
    
    # Control signal
    U = np.zeros((3, 1))
    for i in range(3):
        U[i] = -Umax * S[i] / (abs(S[i]) + k**2 * delta)
    return U

def gain_estimator_bosk(w, wd, dq, delta, gamma, k, Umax):
    """
    LS2125204: Brayan Espinoza
    Input signals:
        w:  Satellite angular rates
        wd: Desired angular rate
        dq: Error quaternion
        delta: Gain
        gamma: Learning rate
        k: Adaptive gain
        Umax: Saturation torque
    """

    dq0 = dq[0]
    dq13 = dq[1:4]
    
    we = w - wd
    
    # Initial variables
    sum_val = 0

    # Sliding surface
    S = we + k**2 * dq13

    # Sum
    for i in range(3):
        sum_val += (we[i] * dq13[i]) / (abs(S[i]) + k**2 * delta) - (abs(we[i]) * (1 + delta)) / (abs(we[i]) + k**2 * (1 + delta))

    # Estimator
    k_dot = gamma * k / (1 + 4 * gamma * (1 - dq0)) * (Umax * sum_val * (we.T @ dq13) - k**2 * (dq13.T @ dq13))
    return k_dot

def boskovic_rk4(dt, qd, wd, wd_ant, q, w, delta, gamma,k, k_ant, k_dot_ant, Umax):
    # Calculate quaternion error
    dq = error_quaternio(qd, q)
    
    # Calculate control law
    k_dot = gain_estimator_bosk(w, wd, dq, delta, gamma, k, Umax)
    
    # Second order Simpson integration
    k = k_ant + dt / 6 * (k_dot_ant + 2 * (k_dot_ant + k_dot) + k_dot)
    
    # Controller
    U = boskovic_control(w, wd, dq, delta, k, Umax)
    print('U: ',U)
    
    # Update parameters for derivation
    k_ant = k
    k_dot_ant = k_dot
    
    return U, k_ant, k_dot_ant