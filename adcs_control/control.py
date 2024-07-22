# Ejemplo de uso
import numpy as np

def skew(vector):
    """
    Devuelve la matriz de skew-symmetric para un vector 3D.
    El vector de entrada debe ser una columna (3x1).
    """
    return np.array([[0, -vector[2, 0], vector[1, 0]],
                     [vector[2, 0], 0, -vector[0, 0]],
                     [-vector[1, 0], vector[0, 0], 0]])

def error_quaternio(qd, q):
    """
    Calcula el error del cuaternión.

    Parámetros:
    qd -- Cuaternión deseado (array-like, [qd0, qd1, qd2, qd3]).
    q -- Cuaternión actual (array-like, [q0, q1, q2, q3]).

    Retorno:
    q_err -- Error del cuaternión (array, [q0_err, q1_err, q2_err, q3_err]).
    """
    # Definición de la matriz Xi
    Xi = np.array([[-qd[1,0], -qd[2,0], -qd[3,0]],
                   [qd[0,0], -qd[3,0], qd[2,0]],
                   [qd[3,0], qd[0,0], -qd[1,0]],
                   [-qd[2,0], qd[1,0], qd[0,0]]])
    
    # Cálculo de los cuaterniones de error
    q13_err = Xi.T @ q
    q0_err = qd.T @ q

    # Combinar parte vectorial con numerica
    q_err = np.concatenate((q0_err, q13_err),axis=0)

    return q_err

def control_feedback(I, x, dq, Wr, Wr_dot, P, K):
    """
    Ley de control realizada mediante Lyapunov con ley de control por Feedback.

    Parámetros:
    I -- Tensor de inercia (cuerpo rígido).
    x -- Vector de estado.
    dq -- Cuaternio de actitud de error.
    Wr -- Velocidades angulares de referencia.
    Wr_dot -- Derivadas de las velocidades angulares de referencia.
    P -- Ganancias de control proporcionales.
    K -- Ganancias de control derivativas.

    Retorno:
    U -- Torque de entrada.
    """
    # Extraer las velocidades angulares del cuerpo del vector de estado
    W = x[4:7]
    
    # Extraer los componentes del cuaternio de error
    dq13 = np.array([dq[1], dq[2], dq[3]])

    # Calcular el error de velocidad angular
    dW = W - Wr
    

    # Calcular la ley de control
    U = -P @ dW - K @ dq13 + skew(W) @ I @ W + I @ (Wr_dot - skew(W) @ Wr)

    return U


def feedback_rk4(dt, qd, wd, wd_ant, q, w, I, P, K):
    x = np.concatenate((q, w), axis=0)
    Wd_dot = (wd-wd_ant)/dt
    dq = error_quaternio(qd, q)
    u = control_feedback(I, x, dq, wd, Wd_dot, P, K)

     # update wd value
    wd_ant = wd
    return u, wd_ant

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

def boskovic_rk4(dt, qd, wd, wd_ant, q, w, delta, gamma, k, k_ant, k_dot_ant, Umax):
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

if __name__ == "__main__":
    # Parámetros de ejemplo
    dt = 0.1  # Paso de tiempo
    qd = np.array([[0], [0], [0], [0]])  # Cuaternión deseado
    wd = np.array([[0.1], [0.2], [0.3]])  # Velocidades angulares deseadas
    wd_ant = np.array([[0.1], [0.2], [0.3]])  # Velocidades angulares deseadas en el paso anterior
    q = np.array([[0.707], [0.707], [0], [0]])  # Cuaternión actual
    w = np.array([[0.01], [0.02], [0.03]])  # Velocidades angulares actuales
    I = np.eye(3)  # Tensor de inercia (identidad para simplificar)
    # Ganancias de control por retroalimentacion
    P = np.eye(3)  # Ganancias de control proporcionales
    K = np.eye(3)  # Ganancias de control derivativas
    # Ganancias de control por boskovick
    delta = 0.01 
    gamma = 0.001
    k = 0
    k_ant = 0
    k_dot_ant = 0
    Umax = 0.1

    # Llamada a la función feedback_rk4
    [u,w_ante] = feedback_rk4(dt, qd, wd, wd_ant, q, w, I, P, K)
    print("Torque de entrada calculado:", u)
    print("w_anterior:", w_ante)
    # Llamada a la función boskovic_rk4
    [U, k_ant, k_dot_ant] = boskovic_rk4(dt, qd, wd, wd_ant, q, w, delta, gamma, k, k_ant, k_dot_ant, Umax)
    print("Torque de entrada calculado Bosk:", U)