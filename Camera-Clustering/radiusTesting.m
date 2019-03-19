close all; clearvars;

% setup
frames = 30;
vMax = 1;
aMax = 1;
radius = calcRadius(frames / 30);
%radius = .03

% .5 = 0.03, 1 = .14, 2 = .56, 3 = 1.26, 4 = 1.92, 5 = 2.5, 6 = 3.12
% fitting results: 
% b =  -0.5547
% m = 0.6109
% data
% xdata = [.5 1 2 3 4 5 6];
% ydata = [.03 .125 .53 1.22 1.89 2.47 3.09];
% fun = @(x,xdata) x(1).*xdata.^3 + x(2).*xdata.^2 + x(3).*xdata + x(4);
% x0 = [0 0 0 0];
% [x,resnorm,residual,exitflag,output] = lsqcurvefit(fun,x0,xdata,ydata);
% 
% quad = @(x,q,a,b,c) q*x.^3 + a*x.^2 + b * x + c; 
% figure;
% scatter(xdata,ydata); hold on;
% plot(0:7,quad(0:7,x(1),x(2),x(3),-0.07)); hold on;
% axis([0 7 0 7]);

%vals = x\y
%line = @(m,x,b) m * x + b;

% add constraints
[curOpt,msg,posOut] = pathOpt2D([5 5], [5+(radius/sqrt(2)) 5+(radius/sqrt(2))],[3 8; 2 9; 1 8]);

if(curOpt(1) <= (frames / 30))
    disp('pass');
end
curOpt(1)   

% helpers
function [radius] = calcRadius(time)
% given an amount of time and a max velocity and acceleration, calculate
% the max distance a particle could travel
% reduction factor
%radius = (factor*vMax * time) + (factor*.5*aMax * time^2);

%ax = sqrt(sqrt(aMax)/2);
%vx = sqrt(sqrt(vMax)/2);
% ax = aMax / 2;
% vx = vMax / 2;
% t = time / factor;
% radius = 0;
% %assume 0 acceleration at beginning and end)
% radius = 2*(vx * t);
% 
% for i=1:factor
% radius = radius + (vx * t + .5 * ax * t^2);
% end


%m = 0.6109;
%b = -0.5547;
x =[-0.0162    0.1837   -0.0038   -0.07];

radius = x(1)*time^3 + x(2)*time^2 + x(3)*time + x(4);

end