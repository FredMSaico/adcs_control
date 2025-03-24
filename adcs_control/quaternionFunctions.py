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

def xi_operator(q):
    Xi = np.array([
        [-q[1], -q[2], -q[3]],
        [ q[0], -q[3],  q[2]],
        [ q[3],  q[0], -q[1]],
        [-q[2],  q[1],  q[0]]
    ])
    return Xi