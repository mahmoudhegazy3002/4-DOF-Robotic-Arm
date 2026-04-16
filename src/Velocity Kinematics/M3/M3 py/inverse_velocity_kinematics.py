#!/usr/bin/env python3
"""
ROS node: inverse_velocity_kinematics.py

Subscribes:
  /desired_ee_velocity  (geometry_msgs/Vector3)  -- desired EE linear velocity [m/s]

Publishes:
  /joint_states_from_ik  (sensor_msgs/JointState) -- name, position (current q), velocity (q_dot)

Notes:
 - Joint vector ordering: [q1, q2, q3, q4]
 - All angles in radians internally.
 - Jacobian computed numerically using finite differences from the forward kinematics X(q).
"""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Link lengths
L1 = 0.06
L2 = 0.04355
L3 = 0.145
L4 = 0.1058
L5 = 0.034
L6 = 0.1003

# Default joint angles (radians)
DEFAULT_Q_DEG = [0.0, 90.0, 0.0, 90.0]  # degrees from your MATLAB example
DEFAULT_Q = np.deg2rad(DEFAULT_Q_DEG)

JOINT_NAMES = ['q1', 'q2', 'q3', 'q4']


def dh_transform(a, alpha, d, theta):
    """Return homogeneous transform for DH parameters (numeric)."""
    ct = np.cos(theta); st = np.sin(theta)
    ca = np.cos(alpha); sa = np.sin(alpha)
    A = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,   sa,     ca,    d],
        [0.0,  0.0,    0.0,  1.0]
    ], dtype=float)
    return A


def forward_kinematics_position(q):
    """
    Compute end-effector position X(q) = [x,y,z] given q = [q1,q2,q3,q4].
    Implements the same DH setup as in your MATLAB code:
      DH = [  0,        pi/2,  L1+L2,       q1;
              L3,       pi,    0,           q2 + pi/2;
              L4+L5,    pi,    0,           q3;
              L6,       0,     0,           q4 ];
    """
    q1, q2, q3, q4 = q
    DH = [
        (0.0,        np.pi/2,  L1+L2,       q1),
        (L3,         np.pi,    0.0,         q2 + np.pi/2),
        (L4+L5,      np.pi,    0.0,         q3),
        (L6,         0.0,      0.0,         q4)
    ]

    T = np.eye(4)
    for a, alpha, d, theta in DH:
        A = dh_transform(a, alpha, d, theta)
        T = T.dot(A)
    pos = T[0:3, 3].reshape((3,))
    return pos


def numerical_jacobian(q, eps=1e-7):
    """
    Numerical Jacobian of X(q) w.r.t q using central finite differences.
    Returns 3x4 Jacobian.
    """
    q = np.asarray(q, dtype=float)
    n = q.size
    J = np.zeros((3, n), dtype=float)
    fx = forward_kinematics_position(q)
    for i in range(n):
        dq = np.zeros_like(q)
        dq[i] = eps
        f_plus = forward_kinematics_position(q + dq)
        f_minus = forward_kinematics_position(q - dq)
        J[:, i] = (f_plus - f_minus) / (2 * eps)
    return J


class InverseVelocityKinematicsNode:
    def __init__(self):
        rospy.init_node('inverse_velocity_kinematics_node', anonymous=False)

        # Parameters
        self.q = np.array(rospy.get_param('~initial_q', list(DEFAULT_Q)), dtype=float)
        if len(self.q) != 4:
            rospy.logwarn("initial_q param length != 4; using defaults")
            self.q = np.array(DEFAULT_Q)

        # Topics
        self.pub = rospy.Publisher('/joint_states_from_ik', JointState, queue_size=1)
        rospy.Subscriber('/desired_ee_velocity', Vector3, self.desired_velocity_cb)

        self.rate_hz = rospy.get_param('~publish_rate', 10)
        self.rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("inverse_velocity_kinematics_node started (publish_rate=%d Hz)", self.rate_hz)

        # Publish an initial joint state periodically (so RViz / other nodes see something)
        self.publish_current_state(np.zeros(4))

    def desired_velocity_cb(self, msg):
        # Desired end-effector linear velocity
        Xd_dot = np.array([msg.x, msg.y, msg.z], dtype=float).reshape((3,))

        # compute Jacobian at current q
        J = numerical_jacobian(self.q)

        # compute pseudo-inverse and joint velocities
        try:
            J_pinv = np.linalg.pinv(J)
            q_dot = J_pinv.dot(Xd_dot)
        except np.linalg.LinAlgError as e:
            rospy.logerr("Jacobian inversion failed: %s", str(e))
            q_dot = np.zeros(4)

        # Publish joint velocities in a JointState message
        self.publish_current_state(q_dot)

        rospy.loginfo("Received desired vel: [% .4f, % .4f, % .4f] -> q_dot: %s",
                      Xd_dot[0], Xd_dot[1], Xd_dot[2],
                      np.array2string(q_dot, precision=4, separator=', '))

    def publish_current_state(self, q_dot):
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = JOINT_NAMES
        js.position = list(self.q)  # we do not integrate q here; this node only publishes velocities
        js.velocity = list(q_dot)
        # Effort left empty
        self.pub.publish(js)

    def spin(self):
        while not rospy.is_shutdown():
            # keep publishing last-known zeros/velocities at rate for visibility
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = InverseVelocityKinematicsNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

