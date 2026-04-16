#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def dh_transform(a, alpha, d, theta):
    """Compute a single DH transformation matrix"""
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics(thetas):
    """Compute the full transformation and joint positions"""
    # Link lengths
    L1 = 0.06
    L2 = 0.04355
    L3 = 0.145
    L4 = 0.1058
    L5 = 0.034
    L6 = 0.1003

    DH = np.array([
        [0,         np.pi/2,  L1+L2,        thetas[0]],
        [L3,        np.pi,    0,            thetas[1] + np.pi/2],
        [L4+L5,     np.pi,    0,            thetas[2]],
        [L6,        0,        0,            thetas[3]]
    ])

    T_total = np.eye(4)
    positions = []

    for i in range(DH.shape[0]):
        a, alpha, d, theta = DH[i]
        T_total = np.dot(T_total, dh_transform(a, alpha, d, theta))
        pos = T_total[0:3, 3]
        positions.append(pos)
        rospy.loginfo(f"Joint {i+1} position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")

    ee = positions[-1]
    rospy.loginfo(f"=== End Effector (Global Frame) ===\nx={ee[0]:.4f}, y={ee[1]:.4f}, z={ee[2]:.4f}")
    return positions, ee

def main():
    rospy.init_node('fk_from_dh', anonymous=True)
    pub = rospy.Publisher('/end_effector_pose', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    rospy.loginfo("Forward Kinematics Node Started...")

    while not rospy.is_shutdown():
        # You can later replace this with ROS topic subscription for real joint angles
        joint_degs = [
            float(input("Joint 1 (deg): ")),
            float(input("Joint 2 (deg): ")),
            float(input("Joint 3 (deg): ")),
            float(input("Joint 4 (deg): "))
        ]

        thetas = np.deg2rad(joint_degs)
        _, ee = forward_kinematics(thetas)

        msg = Float64MultiArray()
        msg.data = [ee[0], ee[1], ee[2]]
        pub.publish(msg)

        rospy.loginfo(f"Published End-Effector Pose: {msg.data}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
