function [rotL,rotF] = moveCamera(line, f, t)
% randomly rotate camera, translate by t
% if t = [0;0], no translation

f = [f(1);f(2)];

% rotation and translation matrix
% rotate in (-pi/2, pi/2) so the cameras face "up" towards the set
theta = (-pi/2) + ((pi/2) - (-pi/2)) * rand(1,1);
R = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];
translation = t; 

% move camera to origin and rotate
shift = repmat(line(:,2),[1 3]);
centeredLine = line - shift;
rotLine = [R * centeredLine(:,1) ...
           R * centeredLine(:,2) ...
           R * centeredLine(:,3)];
centeredF = f - line(:,2);
rotF = R * centeredF;

% move back and apply translation
rotL = rotLine + shift + repmat(translation,[1 3]);
rotF = rotF + line(:,2) + translation;

% testing
% makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;
% figure;
% drawCamera(line,makeLine,[0 10 0 10],f);
% drawCamera(rotL,makeLine,[0 10 0 10],rotF);
% axis([0 10 0 10]);

end

