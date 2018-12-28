function [c,ceq] = constraints2D(x)

global gridN
global maxPosx
global minPosx
global maxPosy
global minPosy
global maxVelocity
global maxAcceleration
global constraints

% Calculate the timestep
sim_time = x(1);
delta_time = sim_time / gridN;

% Get the states / inputs out of the vector
positions(:,1) = x(2 : 1 + gridN);
positions(:,2) = x(2 + gridN : 1 + gridN * 2);
vels(:,1) = x(2 + gridN * 2 : 1 + gridN * 3);
vels(:,2) = x(2 + gridN * 3 : 1 + gridN * 4);
accs(:,1) = x(2 + gridN * 4 : 1 + gridN * 5);
accs(:,2) = x(2 + gridN * 5 : end);

%% Constrain component velocities and accelerations
% for square centered at (5,5), width = 2
% -max(abs(positions(:,1) - 5),abs(positions(:,2) - 5)) + 2]

% no position constraints
if (constraints == 0)
    c = [(maxVelocity * -1 * ones(gridN,1)) - sqrt(vels(:,1).^2 + vels(:,2).^2);     % max vel component constraint
        (maxAcceleration * -1 * ones(gridN,1)) - sqrt(accs(:,1).^2 + accs(:,2).^2)]; % max acc component constraint
else    
    % get lines for cam's fov
    [pt,~,~,slopeL,slopeR,~,~] = makeLines(constraints);
    
    c = [(maxVelocity * -1 * ones(gridN,1)) - sqrt(vels(:,1).^2 + vels(:,2).^2);     % max vel component constraint
        (maxAcceleration * -1 * ones(gridN,1)) - sqrt(accs(:,1).^2 + accs(:,2).^2); % max acc component constraint
        max(positions(:,2),pt(2)) + (slopeL * (max(positions(:,1),pt(1)) - pt(1))) - pt(2);     % left line
        max(positions(:,2),pt(2)) - (slopeR * (max(positions(:,1),pt(1)) - pt(1))) - pt(2)];   % right line
end

%% Enforce smoothness
% Constrain initial position and velocity to be zero
ceq = [positions(1,1) - minPosx;
    positions(1,2) - minPosy;
    vels(1,1);
    vels(1,2)];

for i = 1 : length(positions) - 1
    
    % The state at the beginning of the time int
    x_i = [positions(i,1);
        positions(i,2);
        vels(i,1);
        vels(i,2)];
    
    % What the state should be at the start of the next time int
    x_n = [positions(i+1,1);
        positions(i+1,2);
        vels(i+1,1);
        vels(i+1,2)];
    
    % The time derivative of the state at the beginning of the time int
    xdot_i = [vels(i,1);
        vels(i,2);
        accs(i,1);
        accs(i,2)];
    
    % The time derivative of the state at the end of the time int
    xdot_n = [vels(i+1,1);
        vels(i+1,2);
        accs(i+1,1);
        accs(i+1,2)];
    
    % The end state of the time interval calculated using quadrature
    xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
    
    % Constrain the end state of the current time interval to be
    % equal to the starting state of the next time interval
    ceq = [ceq ; x_n - xend];
end

% Constrain end position to 0 and end velocity to 1
ceq = [ceq ;
    positions(end,1) - maxPosx;
    positions(end,2) - maxPosy;
    vels(end,1);
    vels(end,2)];
end