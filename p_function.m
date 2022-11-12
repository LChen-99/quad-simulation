
function [ p_out, v_out, a_out ] = p_function(t, traj_time, alpha)
%   已知参数多项式系数alpha，带入时间t，求解pos(t) 
%
%
%
%
    n = size(traj_time);
    n = n(2) - 1; % 一共 n 段

    if t >= traj_time(end)
        t = traj_time(end);
    end
    t_index = 1;
    for i = 1:n  % 找到t属于哪一时间段
        if t < traj_time(i)
            break;
        end
        t_index = i;
    end
    s = traj_time(t_index);    %计算S_i-1
    T = traj_time(t_index + 1) - traj_time(t_index); %计算Ti
    
    pos = [1    ((t-s)/T)  ((t-s)/T)^2    ((t-s)/T)^3    ((t-s)/T)^4    ((t-s)/T)^5 ((t-s)/T)^6    ((t-s)/T)^7];
    vel = [0    (1/T)      2*((t-s)/T)/ T    3*((t-s)/T)^2 / T    4*((t-s)/T)^3 /T    5*((t-s)/T)^4/T  6*((t-s)/T)^5 /T    7*((t-s)/T)^6 /T];
    acc = [0      0         2/(T^2)    2*3*((t-s)/T)/(T^2)    3*4*((t-s)/T)^2 /(T^2)    4*5*((t-s)/T)^3/(T^2)    5*6*((t-s)/T)^4 /(T^2)      6*7*((t-s)/T)^5 /(T^2) ] ;
    a = alpha(((t_index - 1) * 8 + 1) : (8 * t_index));
    p_out = pos * a;
    v_out = vel * a;
    a_out = acc * a;
end