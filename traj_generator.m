function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 alpha alpha_x alpha_y alpha_z   last_dx last_dz last_t
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here
if nargin > 2
    last_dx = 0;
    last_dz = 1;
    last_t = 0;
    n = size(waypoints);
    n = n(2);
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0 / 2)];
    waypoints0 = waypoints;
    alpha_x = calculate_alpha(waypoints(1, :), traj_time);
    alpha_y = calculate_alpha(waypoints(2, :), traj_time);
    alpha_z = calculate_alpha(waypoints(3, :), traj_time);
    time = 0:0.01:traj_time(end);
    x = [];
    dx = [];
    ddx = [];
    y = [];
    dy = [];
    ddy = [];
    z = [];
    dz = [];
    ddz = [];
    n = size(time);
    for i = 1 : n(2)
        [xx, dxx, ddxx] = p_function(time(i),traj_time, alpha_x);
        x = [x; xx];
        dx = [dx; dxx];
        ddx = [ddx; ddxx];
        [yy, dyy, ddyy]  = p_function(time(i),traj_time, alpha_y);
        y = [y; yy];
        dy = [dy; dyy];
        ddy = [ddy; ddyy];
        [zz, dzz, ddzz]  = p_function(time(i),traj_time, alpha_z);
        z = [z; zz];
        dz = [dz; dzz];
        ddz = [ddz; ddzz];
    end
%     figure(1);
%     plot(time, z);
%     figure(2);
%     plot(time, dz);
%     figure(3);
%     plot(time, ddz);
%     figure(4);
%     


%     plot3(x, y, z);
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end

    [des_x, des_dx, des_ddx] = p_function(t, traj_time, alpha_x);
    [des_y, des_dy ,des_ddy] = p_function(t, traj_time, alpha_y);
    [des_z, des_dz ,des_ddz] = p_function(t, traj_time, alpha_z);
%     a = 1;
%     t = t / 1.5;
%     T = 0.001;
%     des_x = a*cos(t)/(1 + sin(t)^2);
%     des_dx = -a * (sin(t)*(1+sin(t)^2) + cos(t)*(2*cos(t)*sin(t))) / ((1+sin(t)^2)^2);
%     des_ddx = (des_dx - last_dx) / T;
%     last_dx = des_dx;
%     des_y = 0;
%     des_dy = 0;
%     des_ddy = 0;
%     des_z = a*sin(t).*cos(t)./(1 + sin(t)^2);
%     des_dz = a * (cos(2*t)*(1+sin(t)^2) - 2*sin(t)*cos(t)*sin(t)*cos(t)) / ((1 + sin(t)*sin(t))^2);
%     des_ddz = (des_dz - last_dz) /T ;
%     last_dz = des_dz;
    a = 0.5;
    t = t / 1.5;
    b = 1;
    des_z = a*sin(2*t);
    des_dz = 2*a*cos(2*t);
    des_ddz = -4*a*sin(2*t);
    des_y = 0;
    des_dy = 0;
    des_ddy = 0;
    des_x = b*cos(t);
    des_dx = -b*sin(t);
    des_ddx =  -b*cos(t);
 
    desired_state.pos = [des_x des_y des_z]';
    desired_state.vel = [des_dx des_dy des_dz]';
    desired_state.acc = [des_ddx des_ddy des_ddz]';
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    
end
% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

