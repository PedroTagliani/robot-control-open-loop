from spatialmath import *
from spatialmath.base import *
from math import pi, degrees, radians
import numpy as np
import sympy as sp

# Variables relacionadas a cada una de las longitudes intervinientes en los cálculos (todo en centímetros)
l1 = 14.638
l2 = 12.725
l3 = 10.222
l4 = 11.200
l5 = 8.200

def forward_kinematics(q): # Los valores de las variables articulares están en grados

    # Se pasan los ángulos de grados a radianes
    q1 = radians(q[0])
    q2 = radians(q[1])
    q3 = radians(q[2])
    q4 = radians(q[3])

    # Con la tabla de parámetros DH ya completada, se arman cada una de las MTH:
    A01 = SE3.Rz(q1  - radians(90)) * SE3.Tz(l1) * SE3.Tx(0) * SE3.Rx(radians(90))
    A12 = SE3.Rz(q2) * SE3.Tz(0) * SE3.Tx(l2) * SE3.Rx(0)
    A23 = SE3.Rz(q3  - radians(118)) * SE3.Tz(0) * SE3.Tx(l3) * SE3.Rx(0)
    A34 = SE3.Rz(q4 - radians(20)) * SE3.Tz(0) * SE3.Tx(l4) * SE3.Rx(0)
    A45 = SE3.Ty(-l5) # Se traslada hacia la punta del marcador
    T = A01 * A12 * A23 * A34 * A45

    return T

def forward_kinematics_xyz_euler_repr(q): # q: Lista con los valores de las posiciones articulares en grados
    T = forward_kinematics(q)

    p = T.A[:3,3]
    px, py, pz = p

    # https://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
    if T.A[2,0] != 1 and T.A[2,0] != -1:
        euler_angle_y = -sp.asin(T.A[2,0])
        # euler_angle_y_sol2 = sp.pi - euler_angle_y_sol1
        
        euler_angle_x = sp.atan2(T.A[2,1]/sp.cos(euler_angle_y), T.A[2,2]/sp.cos(euler_angle_y))
        # euler_angle_x_sol2 = sp.atan2(S5.A[2,1]/sp.cos(euler_angle_y_sol2), S5.A[2,2]/sp.cos(euler_angle_y_sol2))
        
        euler_angle_z = sp.atan2(T.A[1,0]/sp.cos(euler_angle_y), T.A[0,0]/sp.cos(euler_angle_y))
        # euler_angle_z_sol2 = sp.atan2(S5.A[1,0]/sp.cos(euler_angle_y_sol2), S5.A[0,0]/sp.cos(euler_euler_angle_y_sol2))
    else:
        euler_angle_z = 0
        
        if T.A[2,0] == -1:
            euler_angle_y = sp.pi/2
            euler_angle_x = euler_angle_z + sp.atan2(T.A[0,1], T.A[0,2])
        else:
            euler_angle_y = -sp.pi/2
            euler_angle_x = -euler_angle_z + sp.atan2(-T.A[0,1], -T.A[0,2])

    euler_angle_x = degrees(euler_angle_x)
    euler_angle_y = degrees(euler_angle_y) # Pitch angle
    euler_angle_z = degrees(euler_angle_z)

    # [px, py, pz, pitchAngle]
    return [round(px,2), round(py,2), round(pz,2), round(euler_angle_y,2)]

if __name__ == '__main__':
    # Ejemplo de aplicación
    q1 = 70
    q2 = 20
    q3 = 40
    q4 = 70.0

    q = [q1, q2, q3, q4]

    T1 = forward_kinematics(q)

    T1_V2 = forward_kinematics_xyz_euler_repr(q)

    print(T1)

    print(T1_V2)