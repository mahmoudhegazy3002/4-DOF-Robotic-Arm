clc; clear; close all;

disp('===== STARTING INVERSE VELOCITY KINEMATICS =====');

syms q1 q2 q3 q4 real

% --- Link Lengths (from Omar's robot data)
L1 = 0.06;
L2 = 0.04355;
L3 = 0.145;
L4 = 0.1058;
L5 = 0.034;
L6 = 0.1003;

% --- DH Table: [a, alpha, d, theta]
DH = [  0,        pi/2,  L1+L2,       q1;
        L3,       pi,    0,           q2 + pi/2;
        L4+L5,    pi,    0,           q3;
        L6,       0,     0,           q4 ];

% --- Compute Transformation Matrices
T = eye(4);
for i = 1:4
    a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4);
    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
    T = simplify(T * A);
end

% --- Forward Kinematics: End Effector Position
X = simplify(T(1:3,4));

disp(' ');
disp('=== Symbolic Forward Kinematics (End-Effector Position) ===');
disp(X);

% --- Jacobian Matrix
J = simplify(jacobian(X, [q1 q2 q3 q4]));

disp(' ');
disp('=== Symbolic Jacobian Matrix ===');
disp(J);

% --- Numerical Evaluation ---
q_deg = [180 90 45 0];
q_rad = deg2rad(q_deg);

J_num = double(subs(J, [q1 q2 q3 q4], q_rad));
X_num = double(subs(X, [q1 q2 q3 q4], q_rad));

disp(' ');
disp('=== End-Effector Position [x y z] (m) ===');
disp(X_num.');

disp(' ');
disp('=== Numerical Jacobian ===');
disp(J_num);

% --- Example Desired End-Effector Velocity ---
Xd_dot = [-0.052; 0.2; 0.03]; % m/s in X, Y, Z directions

% --- Inverse Velocity Kinematics ---
J_pinv = pinv(J_num);
q_dot = J_pinv * Xd_dot;

disp(' ');
disp('=== Joint Velocities [rad/s] ===');
disp(q_dot.');

disp('===== END OF SCRIPT =====');