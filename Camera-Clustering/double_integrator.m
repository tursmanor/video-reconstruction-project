close all; clearvars;
%%
% edited scripts from http://sam.pfrommer.us/tutorial-direct-collocation-trajectory-optimization-with-matlab

global gridN
global maxPos
global minPos
maxPos = 100;
minPos = 10;
gridN = 20;

% Minimize the simulation time
time_min = @(x) x(1);

% The initial parameter guess; 1 second, gridN positions, gridN velocities,
% gridN accelerations;
x0 = [1; 
      linspace(minPos,maxPos,gridN)'; 
      linspace(0,1,gridN)'; 
      ones(gridN, 1) * 5];

% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];

% Bounds: time [0 inf], pos [-inf inf], vel [-5 5], acc [-1 1]
lb = [0; ones(gridN, 1) * -Inf;  ones(gridN,1) * -5; ones(gridN, 1) * -1];
ub = [Inf;  ones(gridN, 1) * Inf; ones(gridN,1) * 5; ones(gridN, 1) * 1];

% Options for fmincon
% TolFun is termination epsilon, default is 1e-6
% MaxIter is max num iterations, default 400
% MaxFunEvals is max num fun evaliations, default 3000
% Display shows iteration results as fun runs
% DiffMinChange is min change in vars for gradients
% Algorithm type is sqp, not for large scale problems- use interior-point
% for large scale
% Plotfcn uses my custom plotting function to plot position per iteration
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 100000, ...%'Display', 'iter', ...
                       'DiffMinChange', 0.001, 'Algorithm', 'sqp', ...
                       'PlotFcn', @plotPos);

% Solve for the best simulation time + control input
optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
                  @double_integrator_constraints, options);
              
% Discretize the times
sim_time = optimal(1);
delta_time = sim_time / gridN;
times = 0 : delta_time : sim_time - delta_time;

% Get the state + accelerations (control inputs) out of the vector
positions = optimal(2             : 1 + gridN);
vels      = optimal(2 + gridN     : 1 + gridN * 2);
accs      = optimal(2 + gridN * 2 : end);

%% Make the plots
% figure();
% plot(times, accs);
% title('Control Input (Acceleration) vs Time');
% xlabel('Time (s)');
% ylabel('Acceleration (m/s^2)');
% figure();
% plot(times, vels);
% title('Velocity vs Time');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
figure();
plot(times, positions);
title('Position vs Time');
xlabel('Time (s)');
ylabel('Position (m)');