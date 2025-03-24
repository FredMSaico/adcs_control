import numpy as np
from quaternionFunctions import skew, error_quaternio

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