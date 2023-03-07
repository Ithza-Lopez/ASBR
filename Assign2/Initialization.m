%% Initialization

% 8 joints
% 0 is the base of robot
% 9 is the end effector

% measurments in [mm]
L0 = 100;
L3 = 265;
L5 = 360;
L7 = 360;
L9 = 175;

%% Body frame Screw Matrix Initialization

omega_m = [ 0,0,0;
            0,0,1;
            0,1,0;
            -1,0,0;
            0,0,-1;
            1,0,0;
            0,0,1;
            1,0,0;
            0,0,1];

vel_m = [0, -L3, L5 + L7 + L9;
        -L0, -L3, L5 + L7 + L9;
         0, 0, L5 + L7 + L9;
         0, 0, L5 + L7 + L9;
         0, 0, 0;
         0, 0, L7 + L9;
         0, 0, 0;
         0, 0, L9;
         0, 0, 0];

theta_m = [0;0;0;0;0;0;0;0;0];

M = [eye(3), [0;-L3;L5+L7+L9]; 0,0,0,1];

save('Body_omega.mat', "omega_m");
save('Body_vel.mat', "vel_m")
save('Body_M.mat', "M")
save('Body_theta.mat', "theta_m")