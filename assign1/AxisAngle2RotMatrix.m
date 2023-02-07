function [Rot_matrix] = AxisAngle2RotMatrix(omega, theta)

%axis angle = [x, y, z, rotation angle]
%first three specifies axis of rotation, last is angle of rotation
axang = [omega,theta];
% correct = axang2rotm(axang);
% disp(correct)
omega_hat = [0 -omega(3) omega(2) ; omega(3) 0 -omega(1) ; -omega(2) omega(1) 0 ];
Rot_matrix = eye(length(omega)) + omega_hat*sin(theta)+ (omega_hat^2)*(1-cos(theta));
end