%% ===== 1. INITIALIZATION & SETUP =====
clc; clear; close all;

% --- Configuration Parameters ---
coord_A      = [0.1217; 0; 0.1];      % Initial Position (x, y, z)
coord_B      = [0.1217; 0; 0.2];      % Target Position (x, y, z)
run_duration = 5;                     % Total movement time (seconds)
delta_t      = 0.01;                  % Sample rate
init_guess   = deg2rad([180;0;250;70]); % Initial IK guess

% --- Calculate Boundary Joint Angles (IK Solved ONLY here) ---
disp("Calculating Inverse Kinematics for boundary points...");
theta_start = compute_inverse_kin(init_guess, coord_A);
theta_end   = compute_inverse_kin(theta_start, coord_B);

% --- Generate the Joint Space Trajectory ---
MotionProfile = generate_joint_path(theta_start, theta_end, run_duration, delta_t);

disp("=== Joint Space Trajectory Generated ===");
disp(["Total steps: " num2str(height(MotionProfile))]);

%% ===== 2. SIMULINK EXPORT =====
% Formatting data into Timeseries objects for Simulink import
sim_time = MotionProfile.Time_S;

% Create timeseries for the 4 active joints
var_theta1 = timeseries(MotionProfile.J1_Rad, sim_time);
var_theta2 = timeseries(MotionProfile.J2_Rad, sim_time);
var_theta3 = timeseries(MotionProfile.J3_Rad, sim_time);
var_theta4 = timeseries(MotionProfile.J4_Rad, sim_time);

% Dummy variable for gripper/5th axis (required by some blocks)
var_theta5 = timeseries(zeros(size(sim_time)), sim_time);

disp(" ");
disp(">> DATA READY FOR SIMULINK <<");
disp("Created variables: var_theta1, var_theta2, var_theta3, var_theta4, var_theta5");

%% ===== 3. CORE TRAJECTORY GENERATOR =====
function TableData = generate_joint_path(q_start, q_final, T_total, dt)
    time_steps = 0:dt:T_total;
    num_steps = length(time_steps);
    num_joints = length(q_start);
    
    % Pre-allocate matrix for speed
    % Columns: [Time, x, y, z, th1, th2, th3, th4, dth1, dth2, dth3, dth4]
    data_log = zeros(num_steps, 12);
    
    % --- Calculate Cubic Polynomial Coefficients ---
    % Formula: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    % Boundary conditions: Velocity starts and ends at 0.
    a0 = q_start;
    a1 = zeros(num_joints, 1);
    a2 = (3 * (q_final - q_start)) / (T_total^2);
    a3 = (-2 * (q_final - q_start)) / (T_total^3);
    
    for k = 1:num_steps
        t_curr = time_steps(k);
        
        % 1. Calculate Joint Positions (Polynomial)
        q_now = a0 + a1*t_curr + a2*(t_curr^2) + a3*(t_curr^3);
        
        % 2. Calculate Joint Velocities (Derivative of Polynomial)
        q_vel = a1 + 2*a2*t_curr + 3*a3*(t_curr^2);
        
        % 3. Calculate Forward Kinematics (Just for visualization/checking)
        xyz_now = compute_forward_kin(q_now);
        
        % Store: [Time, X, Y, Z, q1...q4, qdot1...qdot4]
        data_log(k, :) = [t_curr, xyz_now', q_now', q_vel'];
    end
    
    % Convert to Table with distinct names
    TableData = array2table(data_log, ...
        'VariableNames', {'Time_S', 'X_m', 'Y_m', 'Z_m', ...
                          'J1_Rad', 'J2_Rad', 'J3_Rad', 'J4_Rad', ...
                          'Vel1', 'Vel2', 'Vel3', 'Vel4'});
end

%% ===== 4. KINEMATICS LIBRARY =====

% --- Forward Kinematics (FK) ---
function pos_xyz = compute_forward_kin(angles)
    % Robot Physical Dimensions
    len1 = 0.06; len2 = 0.04355; len3 = 0.145; 
    len4 = 0.1058; len5 = 0.034; len6 = 0.1003;
    
    % Denavit-Hartenberg Table
    % [a, alpha, d, theta]
    dh_params = [ 0,        pi/2,   len1+len2,  angles(1);
                  len3,     pi,     0,          angles(2)+pi/2;
                  len4+len5,pi,     0,          angles(3);
                  len6,     0,      0,          angles(4)];
                  
    TransMat = eye(4);
    
    for i = 1:4
        a_i = dh_params(i,1); 
        alp = dh_params(i,2); 
        d_i = dh_params(i,3); 
        th_i = dh_params(i,4);
        
        T_link = [cos(th_i), -sin(th_i)*cos(alp), sin(th_i)*sin(alp), a_i*cos(th_i);
                  sin(th_i),  cos(th_i)*cos(alp), -cos(th_i)*sin(alp), a_i*sin(th_i);
                  0,          sin(alp),            cos(alp),             d_i;
                  0,          0,                   0,                    1];
        TransMat = TransMat * T_link;
    end
    
    pos_xyz = TransMat(1:3, 4);
end

% --- Numerical Jacobian ---
function JacMat = get_numerical_jacobian(angles)
    perturb = 1e-6;
    n_j = length(angles);
    pos_nominal = compute_forward_kin(angles);
    JacMat = zeros(3, n_j);
    
    for i = 1:n_j
        angles_perturbed = angles;
        angles_perturbed(i) = angles(i) + perturb;
        pos_new = compute_forward_kin(angles_perturbed);
        JacMat(:, i) = (pos_new - pos_nominal) / perturb;
    end
end

% --- Inverse Kinematics (IK) ---
function theta_sol = compute_inverse_kin(theta_guess, target_pos)
    err_tol = 1e-4;
    max_loops = 100;
    learning_rate = 0.3;
    
    theta_sol = theta_guess;
    
    for iter = 1:max_loops
        current_pos = compute_forward_kin(theta_sol);
        error_vec = current_pos - target_pos;
        
        if norm(error_vec) < err_tol
            break;
        end
        
        J_matrix = get_numerical_jacobian(theta_sol);
        
        % Dampened Least Squares / Pseudoinverse update
        theta_sol = theta_sol - learning_rate * pinv(J_matrix) * error_vec;
    end
end