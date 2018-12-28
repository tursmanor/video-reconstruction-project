function [optimal] = pathOpt2D(initPos,endPos,lines)
%% 2d version
% Setup

global gridN
global maxPosx
global minPosx
global maxPosy
global minPosy
global maxVelocity
global maxAcceleration

gridN = 20;
maxPosx = 10;
minPosx = 0;
maxPosy = 10;
minPosy = 0;
maxVelocity = .5;
maxAcceleration = .25;

% Minimize the simulation time
time_min = @(x) x(1);

% no linear constraints
A = [];
b = [];
Aeq = [];
Beq = [];

% Bounds: time [0 inf], pos [0 10], vel [-5 5], acc [-1 1]
lb = [0;
    ones(gridN * 2, 1) * 0;
    ones(gridN * 2, 1) * -5;
    ones(gridN * 2, 1) * -1];
ub = [Inf;
    ones(gridN * 2, 1) * 10;
    ones(gridN * 2, 1) * 5;
    ones(gridN * 2, 1) * 1];

% Options for fmincon
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter',...
                       1000000, 'MaxFunEvals', 1000000, 'DiffMinChange',...
                       0.001, 'Algorithm', 'sqp', 'PlotFcn', @plotPos2D);

%% Run optimization
% Solve for the best simulation time + control input
% [optimal,~,~,out] = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
%                  @constraints2D, options);

% problem = createOptimProblem('fmincon','x0',x0,'lb',lb,'ub',ub,...
%                              'nonlcon',@constraints2D,'objective',time_min);
% ms = MultiStart;
% gs = GlobalSearch(ms);
% [optimal,~,~,out,solns] = run(gs,problem);

%% Brute force global optimization
% 70 converges in 2min for squares
iterations = 10;
bestTime = inf;
tic;
for n=1:iterations
    
    % time, xpos, ypos, xvel, yvel, xacc, yacc
    x0 = [1;
        linspace(minPosx,maxPosx,gridN)' .* rand([gridN 1]);
        linspace(minPosy,maxPosy,gridN)' .* rand([gridN 1]);
        linspace(0,1,gridN)' .* rand([gridN 1]);
        linspace(0,1,gridN)' .* rand([gridN 1]);
        ones(gridN, 1);
        ones(gridN, 1)];
    
    [optimal,~,~,out] = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
        @constraints2D, options);
    
    if(optimal(1) < bestTime)
        bestTime = optimal(1);
        bestSol = optimal;
        %bestOut = out;
    end
    
end
toc
optimal = bestSol;

%% Massage output
% Discretize the times
sim_time = optimal(1);
delta_time = sim_time / gridN;
times = 0 : delta_time : sim_time - delta_time;

% Get the state + accelerations (control inputs) out of the vector
positions(:,1) = optimal(2 : 1 + gridN);
positions(:,2) = optimal(2 + gridN : 1 + gridN * 2);

%% Plot results
% define lines
pt = [6 1];
ptLeft = [5 4];
ptRight = [7 4];
slopeLeft = (ptLeft(2) - pt(2)) / (ptLeft(1) - pt(1));
slopeRight = (ptRight(2) - pt(2)) / (ptRight(1) - pt(1));
leftLine = @(x) slopeLeft * (x - pt(1)) + pt(2);
rightLine = @(x) slopeRight * (x - pt(1)) + pt(2);

figure();
plot(times, positions(:,1));
title('x Position vs Time');
xlabel('Time (s)');
ylabel('x Position (m)');
figure();
plot(times, positions(:,2));
title('y Position vs Time');
xlabel('Time (s)');
ylabel('y Position (m)');
figure();
plot(positions(:,1),positions(:,2));
title('x Position vs y Position');
xlabel('x Position (m)');
ylabel('y Position (m)'); hold on;
plot(0:6,leftLine(0:6)); hold on;
plot(6:10,rightLine(6:10)); hold on;
axis([0 10 0 10]);
%rectangle('Position',[3 3 2 2]);

end