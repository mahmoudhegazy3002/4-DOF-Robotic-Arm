function end_effector_pose()
    clc; clear; close all;

    L1 = 0.06;
    L2 = 0.04355;
    L3 = 0.145;
    L4 = 0.1058;
    L5 = 0.034;
    L6 = 0.1003;

    disp('Forward Kinematics ...');

    theta = deg2rad([ ...
        input('Joint 1 (deg): '), ...
        input('Joint 2 (deg): '), ...
        input('Joint 3 (deg): '), ...
        input('Joint 4 (deg): '), ...
        %input('Joint 5 (deg): '), ...
        %input('Joint 6 (deg): ') ...
    ]);

    DH = [  0,      pi/2,  L1+L2, theta(1);
            L3,     pi,     0,  theta(2)+pi/2;
            L4+L5,     pi,     0,  theta(3);
            L6,     0,       0,   theta(4) 
            ];

    T_total = eye(4);
    for i = 1:size(DH,1)
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); th = DH(i,4);
        Ti = [cos(th), -sin(th)*cos(alpha),  sin(th)*sin(alpha), a*cos(th);
              sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
              0,        sin(alpha),          cos(alpha),          d;
              0,        0,                   0,                   1];
        T_total = T_total * Ti;

      %  fprintf('\nT%d_%d:\n', i-1, i);
        %disp(Ti);
        fprintf('Position of joint %d: x=%.4f, y=%.4f, z=%.4f\n', ...
            i, T_total(1,4), T_total(2,4), T_total(3,4));
    end

    % End-effector position
    ee = T_total(1:3,4);
    fprintf('\n=== End Effector (Global Frame) ===\n');
    fprintf('x = %.4f, y = %.4f, z = %.4f\n\n', ee(1), ee(2), ee(3));
end
