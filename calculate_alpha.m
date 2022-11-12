function [alpha, b] = calculate_alpha(waypoints, traj_time)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
    alpha = [];
    n = size(waypoints);
    n = n(2) - 1;
    
    b = [];
    for i = 1:n
        % getaline(i, k, 0) 第i段， k阶， 0代表时间段开始时刻， 1代表时间段结束
        alpha = [alpha; getaline(i, 0, 0, traj_time); getaline(i, 0, 1, traj_time)];
        b = [b; waypoints(i); waypoints(i + 1)];
    end
    for k = 1:3
        alpha = [alpha; getaline(1, k, 0, traj_time); getaline(n, k, 1, traj_time)];
        b = [b; 0; 0];
    end
    for i = 1:(n-1)
        for k = 1:6
            L1 = getaline(i, k, 1, traj_time);
            L2 = getaline(i+1, k, 0, traj_time);
            L2(8*(i-1) + 1: 8*i) = - L1(8*(i-1) + 1: 8*i);
            alpha = [alpha; L2];
            b = [b; 0];
        end
    end
    alpha = inv(alpha) * b;
end

