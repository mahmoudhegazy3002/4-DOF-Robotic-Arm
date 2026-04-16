clc; clear; close all;
%% === Numeric joint values ===
q = deg2rad([180; 0; 250; 70]);      % initial joint angles
q_dot = deg2rad([0; 0; 0; -15]);             % joint velocities (rad/s)
dt = 0.00001;    % time step (s)
T  = 0.1;     % total simulation time (s) - adjust as needed
N  = T/dt;    % number of steps
%% ===== Initialize end-effector =====
X = end_effector_FK(q);  % initial position
V_trans = zeros(3,1);    % linear velocity
V_rot   = zeros(3,1);    % angular velocity
%% ===== Integrate over time =====
for k = 1:N
    % 1) Compute numerical Jacobian
    J = jacobian_numeric(q);
    
    % 2) Full end-effector velocity
    V_full = J * q_dot;        % 6x1 vector: [linear; angular]
    V_trans = V_full(1:3);     % translational velocity
    V_rot   = V_full(4:6);     % rotational velocity
    % 4) Update joint angles
    q = q + q_dot * dt;
    % 3) Integrate translational velocity to update position
    X =  end_effector_FK(q);
  
   
end

%% ===== Output =====
disp('=== Final End-Effector Position [X Y Z] (m) ===');
disp(X');

disp('=== Final End-Effector Translational Velocity [Vx Vy Vz] (m/s) ===');
disp(V_trans');

disp('=== Final End-Effector Rotational Velocity [Wx Wy Wz] (rad/s) ===');
disp(rad2deg(V_rot'));

%% ===== Functions =====

% ---------- Numerical Jacobian ----------
function J = jacobian_numeric(q)
    % Link lengths
    L1 = 0.06; L2 = 0.04355; L3 = 0.145; L4 = 0.1058; L5 = 0.034; L6 = 0.1003;

    % DH table [a, alpha, d, theta]
    DH = [ 0,      pi/2,  L1+L2,     q(1);
           L3,     pi,    0,         q(2)+pi/2;
           L4+L5,  pi,    0,         q(3);
           L6,     0,     0,         q(4) ];

    % Forward kinematics of each link
    T_all(:,:,1) = eye(4);
    T = eye(4);
    for i = 1:4
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4);
        Ti = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
               0,           sin(alpha),             cos(alpha),            d;
               0,           0,                      0,                     1 ];
        T = T * Ti;
        T_all(:,:,i+1) = T;
    end

    pe = T(1:3,4);  % end-effector position

    % Initialize Jacobian
    Jv = zeros(3,4);
    Jw = zeros(3,4);

    % Build Jacobian numerically
    for i = 1:4
        Ti = T_all(:,:,i);
        oi = Ti(1:3,4);
        zi = Ti(1:3,3);
        Jv(:,i) = cross(zi, pe - oi);  % translational
        Jw(:,i) = zi;                  % rotational
    end

    J = [Jv; Jw];  % 6x4 Jacobian
end

% ---------- Forward Kinematics ----------
function p = end_effector_FK(q)
    L1 = 0.06; L2 = 0.04355; L3 = 0.145; L4 = 0.1058; L5 = 0.034; L6 = 0.1003;

    DH = [ 0,      pi/2,  L1+L2,     q(1);
           L3,     pi,    0,         q(2)+pi/2;
           L4+L5,  pi,    0,         q(3);
           L6,     0,     0,         q(4) ];

    T = eye(4);
    for i = 1:4
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); th = DH(i,4);
        Ti = [ cos(th), -sin(th)*cos(alpha),  sin(th)*sin(alpha), a*cos(th);
               sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
               0,        sin(alpha),          cos(alpha),          d;
               0,        0,                   0,                   1 ];
        T = T * Ti;
    end
    p = T(1:3,4);
end