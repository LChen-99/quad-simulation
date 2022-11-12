function [output] = getaline(i, k, j , traj_time)
%   获得方程A*alpha=b中的A矩阵中的一行， 
%   第i段曲线，的k阶导数， j=0带入i段的开始时间， j=1带入i段的结束时间。
%   k阶导数值 = P * alpha中 ，P是行向量，alpha是系数向量，返回P
    n = size(traj_time);
    n = n(2) - 1; % 一共 n 段

    t_index = i;
    s = traj_time(t_index);    %计算S_i-1
    T = traj_time(t_index + 1) - traj_time(t_index); %计算Ti
    if j == 0
        t = s;
    else
        t = traj_time(t_index + 1);
    end
    M       = ...
       [1    ((t-s)/T)  ((t-s)/T)^2    ((t-s)/T)^3    ((t-s)/T)^4    ((t-s)/T)^5 ((t-s)/T)^6    ((t-s)/T)^7;
        0    (1/T)      2*((t-s)/T)/ T    3*((t-s)/T)^2 / T    4*((t-s)/T)^3 /T    5*((t-s)/T)^4/T  6*((t-s)/T)^5 /T    7*((t-s)/T)^6 /T;
        0      0         2/(T^2)    2*3*((t-s)/T)/(T^2)    3*4*((t-s)/T)^2 /(T^2)    4*5*((t-s)/T)^3/(T^2)    5*6*((t-s)/T)^4 /(T^2)      6*7*((t-s)/T)^5 /(T^2)  ;
        0      0         0           2*3/(T^3)    2*3*4*((t-s)/T) /(T^3)    3*4*5*((t-s)/T)^2/(T^3)    4*5*6*((t-s)/T)^3 /(T^3)      5*6*7*((t-s)/T)^4 /(T^3)  ;
        0      0         0           0              2*3*4 /(T^4)       2*3*4*5*((t-s)/T)/(T^4)    3*4*5*6*((t-s)/T)^2 /(T^4)      4*5*6*7*((t-s)/T)^3 /(T^4)  ;
        0      0         0           0              0        2*3*4*5/(T^5)    2*3*4*5*6*((t-s)/T) /(T^5)      3*4*5*6*7*((t-s)/T)^2 /(T^5)  ;
        0        0           0              0              0        0    2*3*4*5*6 /(T^6)      2*3*4*5*6*7*((t-s)/T) /(T^6)  ];
    output = zeros(1, 8 * n);
   
    N = M(k + 1, :);
    output(8 *(i - 1) + 1: 8 * i) = N;
end

