function [T] = FK_body(M,omega, vel ,theta)
% M : the home configuration matrix
% theta : joint angle matrix [ theta_1; theta_2; theta_3]
% omega : screw axis omega matrix (omega_1, omega_2, omega_3); ...
% vel : screw axis linear velocity matrix [(v_1, v_2,v_3); (v_1, v_2,v_3);..]

e_ST = {};
for joint_num = 1:length(omega)
    omega_i = omega(joint_num, :); %(1,2,3) grab a row of 3
    vel_i = vel(joint_num, :); %(1,2,3) grab a row of 3
    theta_i = theta(joint_num, :); %(1,2,3) grab a row of 3
    skew_omega_i = [0 -omega_i(3) omega_i(2) ; omega_i(3) 0 -omega_i(1) ; -omega_i(2) omega_i(1) 0 ];
    
    e_OT = eye(3) + skew_omega_i * sin(theta_i) + (skew_omega_i)^2 * (1-cos(theta_i));

    if all(omega_i == 0) && norm(vel_i) ==1
        e_ST_i = [eye(3), vel_i'*theta_i;0,0,0,1];
    elseif norm(omega_i) ==1
        p = (eye(3).*theta_i + (1-cos(theta_i))*skew_omega_i + (theta_i - sin(theta_i))*(skew_omega_i)^2)* vel_i';
        e_ST_i = [e_OT, p ;0,0,0,1];
    end 
    e_ST = [e_ST, e_ST_i];
end

T = 1;
for joint_num = 1:length(e_ST)
    T = T.*(e_ST{joint_num});
end 
T = T.*M;
end
