function [outputs,out,positions] = pathOpt2D(initPos,endPos,camConst)
% examples of use:
% camConst = [5 4; 6 2; 7 4]; %left pt, center pt, right pt
% optimal = pathOpt2D(line(:,2)', rotLine(:,2)', 0);
% optimal2 = pathOpt2D(line(:,2)', rotLine(:,2)', camConst);

%% 2d version
% Setup
gridN = 20;
maxPosx = endPos(1);
minPosx = initPos(1);
maxPosy = endPos(2);
minPosy = initPos(2);
maxVelocity = 1;
maxAcceleration = 1;
constraints = camConst;

params = struct('gridN',gridN,'maxPosx',maxPosx,'minPosx',minPosx, ...
                'maxPosy',maxPosy,'minPosy',minPosy,'maxVelocity', ...
                maxVelocity,'maxAcceleration',maxAcceleration, ...
                'constraints',constraints);

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
options = optimoptions(@fmincon,'StepTolerance',1e-2,'OptimalityTolerance',1e-2,'ConstraintTolerance',1e-3,'MaxIterations',2000,'MaxFunctionEvaluations',100000,'Algorithm', 'sqp', 'Display','off');%,'PlotFcn', makePlotPos2D(params));

%% Brute force global optimization
% 70 converges in 2min for squares
% 20 iters @ 20 gridN is between 25-47 seconds on avg in parfor
iterations = 20;
outputs = zeros(gridN * 6 + 1,iterations);

tic
parfor n=1:iterations
    % time, xpos, ypos, xvel, yvel, xacc, yacc
    x0 = [1;
         [minPosx; 10 * rand(gridN-2,1); maxPosx];
         [minPosy; 10 * rand(gridN-2,1); maxPosy];
         linspace(0,1,gridN)' .* rand([gridN 1]);
         linspace(0,1,gridN)' .* rand([gridN 1]);
         ones(gridN, 1);
         ones(gridN, 1)];
    
    [optimal,~,~,out] = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
        makeConstraints2D(params), options);
    
    if (out.message(1:32) == 'Converged to an infeasible point')
        optimal(1) = inf;
    end
    
    outputs(:,n) = optimal;
    outMsg(:,n) = out;
end
toc

[~,bestInd] = min(outputs(1,:));
optimalOut = outputs(:,bestInd);
out = outMsg(:,bestInd);

%% Massage output
% Discretize the times
%sim_time = optimal(1);
%delta_time = sim_time / gridN;
%times = 0 : delta_time : sim_time - delta_time;

% Get the state + accelerations (control inputs) out of the vector
positions(:,1) = optimalOut(2 : 1 + gridN);
positions(:,2) = optimalOut(2 + gridN : 1 + gridN * 2);

%% Plot results
% 
% if (constraints ~= 0)
%     [~,~,~,leftLine,rightLine] = makeLines(camConst);
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