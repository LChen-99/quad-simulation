**matlab 四旋翼仿真**



运行`runsim.m`

*其中controller 和PIDcontroller是分别是se3 和 pid控制器*。



```matlab
%% runsim.m
%% 改这个就可以切换控制器
controlhandle = @controller;


%% pre-calculated trajectories
% trajhandle = @traj_line;
% trajhandle = @traj_helix;
% trajhandle = @traj_helix;

%% Trajectory generation with waypoints
%% You need to implement this
trajhandle = @traj_generator;
waypoints = [0    0    0;
             -1   0    0.5;
             -2   0    0;
             -1   0    -0.5;
             0    0    0;
             1    0    0.5;
             2    0    0;
             1    0    -0.5;
             0    0    0;
             
             -1   0    0.5;
             -2   0    0;
             -1   0    -0.5;
             0    0    0;
             1    0    0.5;
             2    0    0;
             1    0    -0.5;
              ]';
%% traj_generator.m产生轨迹
```

