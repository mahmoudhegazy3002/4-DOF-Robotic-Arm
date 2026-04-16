%% ===== 1. SETUP & PARAMETERS =====
clc; clear;

% Define Parameters
start_point = [0.1217; 0; 0.1];   % Start (x, y, z)
end_point   = [0.1217; 0; 0.2];   % End (x, y, z)
duration    = 5;                  % Total time (seconds)
timestep    = 0.1;               % dt (Changed to 0.1 for smoother motion)
q_guess     = deg2rad([180;0;250;70]); % Initial guess

% Call the function
trajectory_table = calculate_trajectory(start_point, end_point, duration, timestep, q_guess);

% Display the result
disp("=== Trajectory Calculation Complete ===");
disp(["Generated " num2str(height(trajectory_table)) " points of data."]);

%% ===== 2. PREPARE DATA FOR SIMULINK =====
% This section formats the data specifically for your Simulink blocks.

t_time = trajectory_table.Time;

% Create Timeseries objects for the first 4 joints
ts_q1 = timeseries(trajectory_table.q1, t_time);
ts_q2 = timeseries(trajectory_table.q2, t_time);
ts_q3 = timeseries(trajectory_table.q3, t_time);
ts_q4 = timeseries(trajectory_table.q4, t_time);

% Create dummy data for the 5th joint (Gripper/Rotation)
% We set this to 0 because the IK solver only handles 4 joints.
ts_q5 = timeseries(zeros(size(t_time)), t_time);

disp("=== Data is ready for Simulink ===");
disp("Variables created: ts_q1, ts_q2, ts_q3, ts_q4, ts_q5");


%% ===== 3. MAIN ALGORITHM =====
function ResultTable = calculate_trajectory(X0, Xf, Tf, dt, q_init)
    % Time vector
    t_vec = 0:dt:Tf;
    N = length(t_vec);
    
    % Pre-allocate Log
    Log = zeros(N, 11); 
    q_curr = q_init;

    for k = 1:N
        t = t_vec(k);

        % --- Cubic Time Scaling ---
        s = 3*(t/Tf)^2 - 2*(t/Tf)^3;
        sdot = (6/Tf^2)*t - (6/Tf^3)*t^2;

        % --- Task-space trajectory ---
        Xd = X0 + s * (Xf - X0);   % Position
        Vd = sdot * (Xf - X0);     % Linear Velocity

        % --- Inverse Kinematics (Position) ---
        q_curr = local_inverse_kinematics(q_curr, Xd);

        % --- Jacobian Calculation ---
        J = local_compute_jacobian(q_curr);

        % --- Inverse Velocity Kinematics ---
        qdot = pinv(J) * Vd;

        % --- Store Data ---
        Log(k,:) = [Xd', q_curr', qdot'];
    end

    % --- Convert to Table ---
    ResultTable = array2table([t_vec', Log], ...
        'VariableNames', {'Time', 'x', 'y', 'z', ...
                          'q1', 'q2', 'q3', 'q4', ...
                          'qdot1', 'qdot2', 'qdot3', 'qdot4'});
end

%% ===== 4. HELPER FUNCTIONS =====
function X = local_forward_kinematics(q)
    L1 = 0.06; L2 = 0.04355; L3 = 0.145; L4 = 0.1058; L5 = 0.034; L6 = 0.1003;
    
    DH = [ 0,     pi/2, L1+L2, q(1);
           L3,    pi,   0,     q(2)+pi/2;
           L4+L5, pi,   0,     q(3);
           L6,    0,    0,     q(4)];
            
    T = eye(4);
    for i = 1:4
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); th = DH(i,4);
        Ti = [cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha), a*cos(th);
              sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
              0,        sin(alpha),           cos(alpha),            d;
              0,        0,                    0,                     1];
        T = T*Ti;
    end
    X = T(1:3,4);
end

function J = local_compute_jacobian(q)
    delta = 1e-6;
    n = length(q);
    X0 = local_forward_kinematics(q);
    J = zeros(3,n);
    for i=1:n
        dq = zeros(n,1); 
        dq(i) = delta;
        J(:,i) = (local_forward_kinematics(q+dq) - X0)/delta;
    end
end

function q = local_inverse_kinematics(q0, Xd)
    tolerance = 1e-4;
    max_iter  = 100;
    alpha     = 0.3;
    q = q0;
    
    for k = 1:max_iter
        F = local_forward_kinematics(q) - Xd;
        if norm(F) < tolerance
            break;
        end
        J = local_compute_jacobian(q);
        J_inv = pinv(J);
        q = q - alpha * J_inv * F;
    end
end