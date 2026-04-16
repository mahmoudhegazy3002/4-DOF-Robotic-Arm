#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

# ======== (1) Forward Kinematics Function ========
def forward_kinematics_func(q):
    L1, L2, L3, L4, L5, L6 = 0.06, 0.04355, 0.145, 0.1058, 0.034, 0.1003

    DH = np.array([
        [0, np.pi/2, L1+L2, q[0]],
        [L3, np.pi, 0, q[1] + np.pi/2],
        [L4+L5, np.pi, 0, q[2]],
        [L6, 0, 0, q[3]]
    ])

    T = np.eye(4)
    for i in range(4):
        a, alpha, d, th = DH[i]
        Ti = np.array([
            [np.cos(th), -np.sin(th)*np.cos(alpha),  np.sin(th)*np.sin(alpha), a*np.cos(th)],
            [np.sin(th),  np.cos(th)*np.cos(alpha), -np.cos(th)*np.sin(alpha), a*np.sin(th)],
            [0,           np.sin(alpha),             np.cos(alpha),            d],
            [0, 0, 0, 1]
        ])
        T = np.dot(T, Ti)
    return T[0:3, 3]

# ======== (2) Inverse Jacobian Function ========
def inverse_jacobian_matrix(q):
    delta = 1e-6
    n = len(q)
    X0 = forward_kinematics_func(q)
    J = np.zeros((3, n))
    for i in range(n):
        q_delta = np.copy(q)
        q_delta[i] += delta
        X1 = forward_kinematics_func(q_delta)
        J[:, i] = (X1 - X0) / delta
    J_inv = np.linalg.pinv(J)
    return J_inv

# ======== (3) Inverse Kinematics Function ========
def inverse_kinematics_func(q0, Xd):
    tolerance = 1e-4
    max_iter = 100
    alpha = 0.3
    q = np.copy(q0)

    for k in range(max_iter):
        F = forward_kinematics_func(q) - Xd
        if np.linalg.norm(F) < tolerance:
            rospy.loginfo(f"Converged after {k} iterations")
            break
        J_inv = inverse_jacobian_matrix(q)
        q = q - alpha * np.dot(J_inv, F)
    return q

# ======== (4) Main ROS Node ========
def forward_velocity_kinematics_node():
    rospy.init_node('forward_velocity_kinematics_node', anonymous=True)
    pub = rospy.Publisher('/joint_angles', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Desired end-effector position
    Xd = np.array([-0.239, 0, 0.2406])
    q0 = np.deg2rad([0, 30, -15, 90])

    q = inverse_kinematics_func(q0, Xd)
    final_pos = forward_kinematics_func(q)
    error = np.linalg.norm(final_pos - Xd)

    rospy.loginfo("Computed joint angles (deg):")
    rospy.loginfo(np.rad2deg(q))
    rospy.loginfo("Final end-effector position:")
    rospy.loginfo(final_pos)
    rospy.loginfo(f"Position error (m): {error:.6f}")

    msg = Float64MultiArray()
    msg.data = q.tolist()

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        forward_velocity_kinematics_node()
    except rospy.ROSInterruptException:
        pass

