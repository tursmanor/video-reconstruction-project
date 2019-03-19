%% Lessmake some quaternions
clear; close all;

% Read in rotation data
fileID = fopen('../Resources/Output/Rotations.txt','r');
data = fscanf(fileID,'%f',[3 499])';

%% try 1
numRs = size(data,1);
epsilon = 0.00001;
uMatrix = zeros((numRs-1)/3,3);
thetaMatrix = zeros((numRs-1)/3,1);
Theta = zeros((numRs-1)/3,3);
ind = 1;

for i=1:3:(numRs - 1)
 
    % Get into angle-axis rep
    R = data(i:i+2,:);
    u = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    theta = acos(((trace(R) - 1) / 2));
    
    % Sanity check
    if abs(norm(u) - (2*sin(theta))) > epsilon
        disp('error for theta:'+i)
    end
    
    % Sanity check 2
    if (R - R') == zeros(3,3)
        disp('R is symmetric at:'+i)
    end
    
    % Store normalized u
    uMatrix(ind,:) = (u / norm(u))';
    thetaMatrix(ind) = theta;
    Theta(ind,:) = -theta * uMatrix(ind,:);
    ind = ind + 1;
    
end

qMatrix = zeros(size(Theta,1),4);

% Make quaternions
for i=1:size(Theta,1)
    
    curT = Theta(i,:);
    qMatrix(i,:) = [sinc(norm(curT)/2)*curT, cos(norm(curT)/2)];
    
end

% Sanity check, convert quaternions back to R
q = qMatrix(1,:);
x = q(1);
y = q(2);
z = q(3);
w = q(4);

Rnew = [1-(2*y^2)-(2*z^2), (2*x*y)-(2*z*w), (2*x*z)+(2*y*w); 
        (2*x*y)+(2*z*w), 1-(2*x^2)-(2*z^2), (2*y*z)-(2*x*w);
        (2*x*z)-(2*y*w), (2*y*z)+(2*x*w), 1-(2*x^2)-(2*y^2)];
    
 %% try 2
numRs = size(data,1);
qMatrix2 = zeros((numRs-1)/3,4);
ind = 1;

for i=1:3:(numRs - 1)
 
    R = data(i:i+2,:);
    a = R(1,1);
    i = R(3,3);
    e = R(2,2);
    b = R(1,2);
    
    z = sqrt(0.5*((-a/2) + 0.5 + (i/2) - (e/2)));
    y = sqrt(0.25*(-a + 1 - i + e));
    x = sqrt(0.5*((-i/2) + 0.5 + (a/2) - (e/2)));
    w = ((2*x*y) - b) / (2*z);
    
    qMatrix2(ind,:) = [x y z w];
    ind = ind + 1;
end

q = qMatrix2(1,:);
x = q(1);
y = q(2);
z = q(3);
w = q(4);

Rnew2 = [1-(2*y^2)-(2*z^2), (2*x*y)-(2*z*w), (2*x*z)+(2*y*w); 
        (2*x*y)+(2*z*w), 1-(2*x^2)-(2*z^2), (2*y*z)-(2*x*w);
        (2*x*z)-(2*y*w), (2*y*z)+(2*x*w), 1-(2*x^2)-(2*y^2)];
    
    
Rinit = data(1:3,:);
a = Rinit(1,1);
b = Rinit(1,2);
c = Rinit(1,3);
d = Rinit(2,1);
e = Rinit(2,2);
f = Rinit(2,3);
g = Rinit(3,1);
h = Rinit(3,2);
i = Rinit(3,3);

% try 3
A = [2*z,2*w,0,0;
     2*y,0,2*w,0;
     0,2*z,0,-2*x;
     2*z,-2*w,0,0];
new = A \ [c;d;f;g];

x1 = new(1); 
y1 = new(2); 
z1 = new(3); 
w1 = new(4);

Rnew3 = [1-(2*y1^2)-(2*z1^2), (2*x1*y1)-(2*z1*w1), (2*x1*z1)+(2*y1*w1); 
        (2*x1*y1)+(2*z1*w1), 1-(2*x1^2)-(2*z1^2), (2*y1*z1)-(2*x1*w1);
        (2*x1*z1)-(2*y1*w1), (2*y1*z1)+(2*x1*w1), 1-(2*x1^2)-(2*y1^2)];

%% compare
Rinit
Rnew
Rnew2
Rnew3

useASolver(Rinit)

function q = useASolver(R)

syms x y z w;
a = R(1,1);
b = R(1,2);
c = R(1,3);
d = R(2,1);
e = R(2,2);
f = R(2,3);
g = R(3,1);
h = R(3,2);
i = R(3,3);

eqns = [1 - (2*y^2) - (2*z^2) == a,
        (2*x*y) - (2*z*w) == b,
        (2*x*z) + (2*y*w) == c,
        (2*x*y) + (2*z*w) == d,
        1 - (2*x^2) - (2*z^2) == e,
        (2*y*z) - (2*x*w) == f,
        (2*x*z) - (2*y*w) == g,
        (2*y*z) + (2*x*w) == h,
        1 - (2*x^2) - (2*y^2) == i];
vars = [x y z w];

q = solve(eqns,vars);
q.x;


end