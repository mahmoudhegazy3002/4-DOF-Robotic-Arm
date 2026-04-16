clc; clear; close all;

%% ===== Desired end-effector position =====
Xd = [0; 0; 0];  % desired position (X, Y, Z)
q0 = deg2rad([0; 0; 0; 0]);  % initial guess for joint angles

%% ===== Inverse Kinematics Calculation =====
q = inverse_kinematics_func(q0, Xd);

%% ===== Display Results =====
fprintf('Computed joint angles (deg):\n');
disp(rad2deg(q));

fprintf('Final end-effector position:\n');
final_pos = forward_kinematics_func(q);
disp(final_pos);

disp('Position error (m):');
disp(norm(final_pos - Xd));
%% ===== ===== FUNCTIONS ===== =====

% ===== (1) Forward Kinematics Function =====
function X = forward_kinematics_func(q)
    % Robot link lengths (constants)
    L1 = 0.06; L2 = 0.04355; L3 = 0.145; L4 = 0.1058; L5 = 0.034; L6 = 0.1003;

    % Denavit–Hartenberg table
    DH = [ 0, pi/2, L1+L2, q(1);
           L3, pi, 0, q(2)+pi/2;
           L4+L5, pi, 0, q(3);
           L6, 0, 0, q(4)];
    T = eye(4);
    for i = 1:4
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); th = DH(i,4);
        Ti = [cos(th), -sin(th)*cos(alpha),  sin(th)*sin(alpha), a*cos(th);
              sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
              0,        sin(alpha),          cos(alpha),         d;
              0,        0,                   0,                  1];
        T = T * Ti;
    end
    X = T(1:3,4);  % end-effector position
end

% ===== (2) Inverse Jacobian Function =====
function J_inv = inverse_jacobian_matrix(q)
    % Fixed robot link lengths
    L1 = 0.06; L2 = 0.04355; L3 = 0.145; L4 = 0.1058; L5 = 0.034; L6 = 0.1003;

    delta = 1e-6;
    n = length(q);
    X0 = forward_kinematics_func(q);
    J = zeros(3, n);
    for i = 1:n
        q_delta = q;
        q_delta(i) = q_delta(i) + delta;
        X1 = forward_kinematics_func(q_delta);
        J(:, i) = (X1 - X0) / delta;  % numerical differentiation
    end
    J_inv = pinv(J);  % pseudo-inverse
end

% ===== (3) Inverse Kinematics Function =====
function q = inverse_kinematics_func(q0, X)
    tolerance = 1e-4;
    max_iter = 100;
    alpha = 0.3;  % step size
    q = q0;
    for k = 1:max_iter
        F = forward_kinematics_func(q) - X;
        if norm(F) < tolerance
            break;
        end
        J_inv = inverse_jacobian_matrix(q);
        q = q - alpha * J_inv * F;
    end
    fprintf('Iterations: %d\n \n', k);
end