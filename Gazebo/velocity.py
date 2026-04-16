#!/usr/bin/env python3
import rospy
import time
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest, GetLinkState
from std_msgs.msg import Float64MultiArray
import numpy as np

# ======= JOINT AND MODEL CONFIGURATION ==========
JOINTS = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]
MODEL_NAME = "my_robot"
EE_LINK_NAME = "EE_frame"

# ======== FORWARD KINEMATICS ========
def forward_kinematics_func(q):
    """
    Forward kinematics using DH parameters
    Args:
        q: Joint angles [q1, q2, q3, q4] (first 4 joints only)
    Returns:
        End-effector position [x, y, z]
    """
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

# ======== INVERSE JACOBIAN ========
def inverse_jacobian_matrix(q):
    """
    Compute pseudo-inverse of Jacobian matrix
    Args:
        q: Joint angles [q1, q2, q3, q4]
    Returns:
        Pseudo-inverse of Jacobian (4x3)
    """
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

# ======== INVERSE KINEMATICS ========
def inverse_kinematics_func(q0, Xd):
    """
    Solve IK using Newton-Raphson method
    Args:
        q0: Initial guess [q1, q2, q3, q4, q5] or [q1, q2, q3, q4]
        Xd: Desired end-effector position [x, y, z]
    Returns:
        Joint solution [q1, q2, q3, q4, q5] (5 joints)
    """
    tolerance = 1e-4
    max_iter = 10
    alpha = 0.3
    
    # Use first 4 joints for IK
    q = np.copy(q0[:4])
    
    for k in range(max_iter):
        F = forward_kinematics_func(q) - Xd
        if np.linalg.norm(F) < tolerance:
            rospy.loginfo(f"Converged after {k} iterations")
            break
        J_inv = inverse_jacobian_matrix(q)
        q = q - alpha * np.dot(J_inv, F)
    
    # Add Joint_5 (keep from initial guess or default)
    if len(q0) > 4:
        q5 = q0[4]
    else:
        q5 = np.pi/2  # Default 90 degrees
    
    q_full = np.hstack([q, q5])  # 5 joints
    return q_full

# ======== GAZEBO INTERFACE ========
def send_positions(pos_dict):
    """Send joint positions directly to Gazebo"""
    try:
        rospy.wait_for_service('/gazebo/set_model_configuration')
        set_pos = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        req = SetModelConfigurationRequest()
        req.model_name = MODEL_NAME
        req.urdf_param_name = ""
        req.joint_names = JOINTS
        req.joint_positions = [pos_dict[j] for j in JOINTS]
        set_pos(req)
    except Exception as e:
        rospy.logerr(f"Error sending positions: {e}")

def get_ee_position():
    """Get EE position from Gazebo"""
    try:
        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp = get_link_state(EE_LINK_NAME, "world")
        pos = resp.link_state.pose.position
        return np.array([pos.x, pos.y, pos.z])
    except Exception as e:
        rospy.logerr(f"Error getting EE state: {e}")
        return None

# ======== MAIN PROGRAM ========
if __name__ == "__main__":
    rospy.init_node("cartesian_waypoints_driver")
    
    # ====== PUBLISHER ======
    joint_pub = rospy.Publisher("/joint_angles", Float64MultiArray, queue_size=10)
    
    # ====== USER INPUT FOR CARTESIAN WAYPOINTS ======
    N = int(input("Enter number of waypoints: "))
    cartesian_waypoints = []
    
    for i in range(N):
        print(f"\n--- Waypoint {i+1} ---")
        x = float(input("X: "))
        y = float(input("Y: "))
        z = float(input("Z: "))
        cartesian_waypoints.append([x, y, z])
    
    # Joint speed
    speed = float(input("\nEnter joint speed (rad/s): "))
    
    # IK initial guess
    print("\nEnter initial guess for IK solver (degrees):")
    print("(Press Enter to use default: [180, 0, -115, 70, 0])")
    use_default = input("Use default? (y/n) [y]: ").strip().lower()
    
    if use_default == 'n':
        q0_deg = []
        for j in JOINTS:
            angle = float(input(f"  {j}: "))
            q0_deg.append(angle)
        q0 = np.deg2rad(q0_deg)
    else:
        q0 = np.deg2rad([180, 0, -115, 70, 0])
    
    print(f"\nIK initial guess: {np.rad2deg(q0).round(1).tolist()} deg")
    
    # ====== SOLVE IK FOR ALL WAYPOINTS ======
    print("\n========================================")
    print("    SOLVING INVERSE KINEMATICS")
    print("========================================\n")
    
    waypoints = []  # List of joint angle dictionaries
    q_current = q0.copy()
    
    for i, (x, y, z) in enumerate(cartesian_waypoints):
        print(f"Waypoint {i+1}/{N}: ({x}, {y}, {z})")
        
        Xd = np.array([x, y, z])
        q_solution = inverse_kinematics_func(q_current, Xd)
        
        # Verify solution
        final_pos = forward_kinematics_func(q_solution[:4])
        error = np.linalg.norm(final_pos - Xd)
        
        print(f"  Joint angles (deg): {np.rad2deg(q_solution).round(1).tolist()}")
        print(f"  Achieved position: {final_pos.round(4)}")
        print(f"  Position error: {error:.6f} m\n")
        
        # Convert to dictionary
        wp = {JOINTS[j]: q_solution[j] for j in range(len(JOINTS))}
        waypoints.append(wp)
        
        # Use this solution as initial guess for next waypoint
        q_current = q_solution.copy()
    
    print("========================================")
    print("    IK SOLUTIONS COMPLETE")
    print("========================================\n")
    
    # ====== INITIALIZE ======
    pos = {j: 0.0 for j in JOINTS}
    MOVE_TIME = 0.1
    rate = rospy.Rate(1 / MOVE_TIME)
    
    # EE velocity estimation
    ee_last = get_ee_position()
    t_last = time.time()
    
    rospy.loginfo("Starting multi-waypoint motion...")
    
    # ====== MAIN LOOP OVER WAYPOINTS ======
    for idx, target_wp in enumerate(waypoints):
        rospy.loginfo(f"\nGoing to waypoint {idx+1}/{N} ...")
        rospy.loginfo(f"  Target: {cartesian_waypoints[idx]}")
        
        finished = {j: False for j in JOINTS}
        
        while not rospy.is_shutdown():
            all_done = True
            
            # Move joints toward this waypoint
            for j in JOINTS:
                target = target_wp[j]
                diff = target - pos[j]
                
                if abs(diff) > 0.001:
                    step = speed * MOVE_TIME
                    step = min(step, abs(diff))
                    pos[j] += step if diff > 0 else -step
                    all_done = False
                else:
                    pos[j] = target
                    finished[j] = True
            
            send_positions(pos)
            
            # ====== PUBLISH JOINT ANGLES ======
            msg = Float64MultiArray()
            msg.data = [pos[j] for j in JOINTS]
            joint_pub.publish(msg)
            
            # ====== PRINT FOR DEBUG ======
            print(f"\rJoint Angles: {[round(pos[j], 3) for j in JOINTS]}", end="")
            
            # ====== EE POSITION AND VELOCITY ======
            ee_now = get_ee_position()
            t_now = time.time()
            dt = t_now - t_last
            
            if ee_now is not None and ee_last is not None:
                ee_vel = (ee_now - ee_last) / dt
                ee_speed = np.linalg.norm(ee_vel)
                print(f"  |  EE pos = {ee_now.round(3)}  |  EE vel = {ee_vel.round(3)}  |  Speed = {ee_speed:.4f} m/s", end="")
            
            ee_last = ee_now
            t_last = t_now
            
            if all_done:
                print()  # New line
                rospy.loginfo(f"Reached waypoint {idx+1}")
                
                # Verify final position
                if ee_now is not None:
                    target_pos = np.array(cartesian_waypoints[idx])
                    final_error = np.linalg.norm(ee_now - target_pos)
                    rospy.loginfo(f"  Final EE position: {ee_now.round(4)}")
                    rospy.loginfo(f"  Target position: {target_pos.round(4)}")
                    rospy.loginfo(f"  Position error: {final_error*1000:.2f} mm")
                
                break
            
            rate.sleep()
    
    rospy.loginfo("\n========================================")
    rospy.loginfo("All waypoints reached successfully!")
    rospy.loginfo("========================================")
