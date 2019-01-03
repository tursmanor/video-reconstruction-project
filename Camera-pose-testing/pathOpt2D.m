function [optimal,out] = pathOpt2D(initPos,endPos,camConst)
% examples of use:
% camConst = [5 4; 6 2; 7 4]; %left pt, center pt, right pt
% optimal = pathOpt2D(line(:,2)', rotLine(:,2)', 0);
% optimal2 = pathOpt2D(line(:,2)', rotLine(:,2)', camConst);

%% 2d version
% Setup

global gridN
global maxPosx
global minPosx
global maxPosy
global minPosy
global maxVelocity
global maxAcceleration
global constraints

gridN = 20;
maxPosx = endPos(1);
minPosx = initPos(1);
maxPosy = endPos(2);
minPosy = initPos(2);
maxVelocity = .5;
maxAcceleration = .25;
constraints = camConst;

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
    0.001, 'Algorithm', 'sqp');%, 'PlotFcn', @plotPos2D);

%% Brute force global optimization
% 70 converges in 2min for squares
iterations = 5;%10;
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

% if (constraints ~= 0)
%     [~,~,~,~,~,leftLine,rightLine] = makeLines(camConst);
%     
%     figure();
%     plot(positions(:,1),positions(:,2));
%     title('x Position vs y Position');
%     xlabel('x Position (m)');
%     ylabel('y Position (m)'); hold on;
%     plot(0:camConst(1,1),leftLine(0:camConst(1,1))); hold on;
%     plot(camConst(3,1):10,rightLine(camConst(3,1):10)); hold on;
%     axis([0 10 0 10]);
%     
% else
%     figure();
%     plot(positions(:,1),positions(:,2));
%     title('x Position vs y Position');
%     xlabel('x Position (m)');
%     ylabel('y Position (m)'); hold on;
%     axis([0 10 0 10]);
% end

end