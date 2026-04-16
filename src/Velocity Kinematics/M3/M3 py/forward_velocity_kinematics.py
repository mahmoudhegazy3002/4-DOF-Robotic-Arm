#!/usr/bin/env python3
import rospy
import sympy as sp

def jacobian_matrix(q):
    # --- Link lengths (m)
    L1 = 0.06
    L2 = 0.04355
    L3 = 0.145
    L4 = 0.1058
    L5 = 0.034
    L6 = 0.1003

    # --- DH parameters: [a, alpha, d, theta]
    DH = [
        [0, sp.pi/2, L1 + L2, q[0]],
        [L3, sp.pi,   0,      q[1] + sp.pi/2],
        [L4 + L5, sp.pi, 0,   q[2]],
        [L6, 0, 0, q[3]]
    ]

    # --- Forward transformations
    T_total = sp.eye(4)
    Ts = []

    for i in range(4):
        a, alpha, d, th = DH[i]
        Ti = sp.Matrix([
            [sp.cos(th), -sp.sin(th)*sp.cos(alpha),  sp.sin(th)*sp.sin(alpha), a*sp.cos(th)],
            [sp.sin(th),  sp.cos(th)*sp.cos(alpha), -sp.cos(th)*sp.sin(alpha), a*sp.sin(th)],
            [0,           sp.sin(alpha),             sp.cos(alpha),             d],
            [0,           0,                         0,                         1]
        ])
        T_total = sp.simplify(T_total * Ti)
        Ts.append(T_total)

    pe = T_total[0:3, 3]
    Jv = sp.zeros(3, 4)
    Jw = sp.zeros(3, 4)

    for i in range(4):
        Ti = sp.eye(4) if i == 0 else Ts[i-1]
        oi = Ti[0:3, 3]
        zi = Ti[0:3, 2]

        Jv[:, i] = sp.simplify(sp.Matrix.cross(zi, pe - oi))
        Jw[:, i] = zi

    J = sp.simplify(sp.Matrix.vstack(Jv, Jw))
    return J


def forward_velocity_kinematics(q, q_dot):
    J = jacobian_matrix(q)
    V_F = sp.simplify(J * q_dot)
    return V_F


def main():
    rospy.init_node('fvk_solver', anonymous=True)
    rospy.loginfo("=== STARTING FORWARD VELOCITY KINEMATICS ===")

    # Symbolic joint variables
    q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4', real=True)
    q1_dot, q2_dot, q3_dot, q4_dot = sp.symbols('q1_dot q2_dot q3_dot q4_dot', real=True)
    q = sp.Matrix([q1, q2, q3, q4])
    q_dot = sp.Matrix([q1_dot, q2_dot, q3_dot, q4_dot])

    # Compute Jacobian
    J = jacobian_matrix(q)
    rospy.loginfo("\n=== Symbolic Jacobian Matrix J ===\n" + str(J))

    # Compute Forward Velocity
    V_F = forward_velocity_kinematics(q, q_dot)
    rospy.loginfo("\n=== Symbolic Forward Velocity V_F ===\n" + str(V_F))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

