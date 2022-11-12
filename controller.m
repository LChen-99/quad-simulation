function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
m = params.mass;
I = params.I;
g = params.gravity;
maxF = params.maxF;
minF = params.minF;
invI = params.invI;
L = params.arm_length;

% SE(3)
kp_x = 18; kp_y = 32; kp_z = 32;
kd_x = 8; kd_y = 10; kd_z = 10;
kp_phi = 800; kp_theta = 800; kp_psi = 800;
kd_phi = 300; kd_theta = 300; kd_psi = 300;

des_psi = des_state.yaw; des_dpsi = des_state.yawdot;
des_dx = des_state.vel(1);des_dy = des_state.vel(2);des_dz = des_state.vel(3);
des_ddx = des_state.acc(1);des_ddy = des_state.acc(2);des_ddz = des_state.acc(3);
des_x = des_state.pos(1);des_y = des_state.pos(2);des_z = des_state.pos(3);
%rnd_position = normrnd(0,0.1,[2 3]);
%rnd_attitude = normrnd(0,0.1,[2 3]);
% rdm = normrnd(0, 0.1, [4, 3]) / 100;
% x = state.pos(1) + rdm(1, 1) ;y = state.pos(2) + rdm(1, 2);z = state.pos(3) + rdm(1, 3);
% dx = state.vel(1) + + rdm(2, 1);dy = state.vel(2) + rdm(2, 2);dz = state.vel(3) + rdm(2, 3);
% phi = state.rot(1) + rdm(3, 1);theta = state.rot(2) + rdm(3, 2);psi = state.rot(3) + rdm(3, 3);
% dphi = state.omega(1) + rdm(4, 1);dtheta = state.omega(2) + rdm(4, 2); dpsi = state.omega(3) + rdm(4, 3);

x = state.pos(1) ;y = state.pos(2);z = state.pos(3);
dx = state.vel(1);dy = state.vel(2);dz = state.vel(3);
phi = state.rot(1);theta = state.rot(2);psi = state.rot(3);
dphi = state.omega(1);dtheta = state.omega(2); dpsi = state.omega(3);

% Thrust

Fx = m * des_ddx + m * kd_x *(des_dx - dx) + m * kp_x * (des_x - x);
Fy = m * des_ddy + m * kd_y *(des_dy - dy) + m * kp_y * (des_y - y);
Fz = m * des_ddz + m * kd_z *(des_dz - dz) + m * kp_z * (des_z - z) + m * g;
F_des = [Fx; Fy; Fz];
R = RPYtoRot_ZXY(phi, theta, psi);
wR_b = transpose(R);
F = dot(F_des,  wR_b(:, 3));
zb_des = F_des / norm(F_des);
xc_des = [cos(des_state.yaw); sin(des_state.yaw); 0];
yb_des = cross(zb_des, xc_des) / norm(cross(zb_des, xc_des));
xb_des = cross(yb_des, zb_des);
xb_des = xb_des / norm(xb_des);
R_des1 = [xb_des, yb_des, zb_des];
R_des2 = [-xb_des, -yb_des, zb_des];
[r1, p1, y1] = RotToRPY(transpose(R_des1) * R);
e1 = [r1, p1, y1];
[r2, p2, y2] = RotToRPY(transpose(R_des2) * R);
e2 = [r2, p2, y2];
if norm(e1) > norm(e2)
    R_des = R_des2;
else
    R_des = R_des1;
end

[des_phi,des_theta,des_psi] = RotToRPY(R_des);
if F > maxF
    F = maxF;
elseif F < minF
    F = minF;
end

% Moment

%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%   des_state.yaw,
%   des_state.yawdot
des_dphi = 0;
des_dtheta = 0;
R_ab = [cos(theta),0,-cos(phi)*sin(theta);0,1,sin(phi);sin(theta),0,cos(phi)*cos(theta)];
W_xyz=R_ab*[dphi;dtheta;dpsi];
dphi = W_xyz(1);dtheta= W_xyz(2);
dpsi= W_xyz(3);
% e_R = rotationMatrixToVector(transpose(R_des) * wR_b - transpose(wR_b) * R_des) / 2;
drot = state.omega;
ddphi = kp_phi * (des_phi - phi) + kd_phi * (des_dphi - dphi);
ddtheta = kp_theta * (des_theta - theta) + kd_theta * (des_dtheta - dtheta);
ddpsi = kp_psi * (des_psi - psi) + kd_psi * (des_dpsi - dpsi);

ddrot = [ddphi; ddtheta; ddpsi];

% drot = [W_xyz(1); W_xyz(2) ;W_xyz(3)];
M = I * ddrot + cross(drot, I * drot);
% =================== Your code ends here ===================

end
