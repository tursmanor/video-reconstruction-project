% implementation of kukelova et al. 2013
% input: 5 2d-3d point correspondences
% output: radial distortion, f
% assume pp is known, no skew

% 2d pts [ui vi 1]T
% 3d pts [Xi Yi Zi 1]T
% Mv = 0
% M is 5x8, where each row has the form:
% -viXi -viYi -viZi -vi uiXi uiYi uiZi ui
function POut = pnp5(points2D, points3D)
u = points2D(:,1);
v = points2D(:,2);
X = points3D(:,1);
Y = points3D(:,2);
Z = points3D(:,3);

M = [-v.*X -v.*Y -v.*Z -v u.*X u.*Y u.*Z u];

% get null space basis vectors
nullBasis = null(M);
n1 = nullBasis(:,1);
n2 = nullBasis(:,2);
n3 = nullBasis(:,3);

% make sylvester matrix. first get two polynomials using the nullspace
% basis vectors, one in terms of y1 and one in terms of y2
A1 = n1(1)*n1(5) + n1(2)*n1(6) + n1(3)*n1(7);
B1 = n2(1)*n2(5) + n2(2)*n2(6) + n2(3)*n2(7);
C1 = n1(1)*n2(5) + n2(1)*n1(5) + n1(2)*n2(6) + n2(2)*n1(6) + n1(3)*n2(7) + n2(3)*n1(7);
D1 = n1(1)*n3(5) + n1(5)*n3(1) + n1(2)*n3(6) + n1(6)*n3(2) + n1(3)*n3(2) + n1(7)*n3(3);
E1 = n2(1)*n3(5) + n2(5)*n3(1) + n2(2)*n3(6) + n2(6)*n3(2) + n2(3)*n3(7) + n2(7)*n3(3);
F1 = n3(1)*n3(5) + n3(2)*n3(6) + n3(3)*n3(7);

A2 = n1(1)^2 + n1(2)^2 + n1(3)^2 - n1(5)^2 - n1(6)^2 - n1(7)^2;
B2 = n2(1)^2 + n2(2)^2 + n2(3)^2 - n2(5)^2 - n2(6)^2 - n2(7)^2;
C2 = 2*n1(1)*n2(1) + 2*n1(2)*n2(2) + 2*n1(3)*n2(3) - 2*n1(5)*n2(5) - 2*n1(6)*n2(6) - 2*n1(7)*n2(7);
D2 = 2*n1(1)*n3(1) + 2*n1(2)*n3(2) + 2*n1(3)*n3(3) - 2*n1(5)*n3(5) - 2*n1(6)*n3(6) - 2*n1(7)*n3(7);
E2 = 2*n2(1)*n3(1) + 2*n2(2)*n3(2) + 2*n2(3)*n3(3) - 2*n2(5)*n3(5) - 2*n2(6)*n3(6) - 2*n2(7)*n3(7);
F2 = n3(1)^2 + n3(2)^2 + n3(3)^2 - n3(5)^2 - n3(6)^2 - n3(7)^2;

syms y1 y2

a0 = (B1 * y2^2) + (E1 * y2) + F1;
a1 = (C1 * y2) + D1;
a2 = A1;

b0 = (B2 * y2^2) + (E2 * y2) + F2;
b1 = (C2 * y2) + D2;
b2 = A2;

sylvesterMatrix = [a2 a1 a0 0; 0 a2 a1 a0; b2 b1 b0 0; 0 b2 b1 b0];

y2Results = vpa(solve(det(sylvesterMatrix),y2),3);
y2Results = real(y2Results);

% calculate y1 now that we have y2 candidates, using the first equation
y1Results = zeros(size(y2Results,1),2);
for i=1:length(y2Results)
   
    curY2 = y2Results(i);
    temp = vpa(solve(a2*y1^2 + (C1*curY2 + D1)*y1 + (B1*curY2^2 + E1*curY2 + F1) == 0, y1),3);
    %disp(temp);
    y1Results(i,:) = real(temp);

end

% calculate potential options for first two rows of P
% each col is a separate guess for the elements p11-p24
numCombos = size(y2Results,1) * size(y1Results,2);
potentialP = zeros(8,numCombos);

for i=1:length(y2Results)
   potentialP(:,i) = y1Results(i,1)*n1 + y2Results(i)*n2 + n3;
   potentialP(:,i+length(y2Results)) = y1Results(i,2)*n1 + y2Results(i)*n2 + n3;
end

% remove repetitions
potentialP = unique(potentialP','rows')';

%% testing stuff for one instance of first two rows of P
% now calculate last row of P
range = 1:size(potentialP,2);
POut = zeros(3,4,range(end));

for curP = range

constraints = [potentialP(1,curP) potentialP(2,curP) potentialP(3,curP) 0; ...
               potentialP(5,curP) potentialP(6,curP) potentialP(7,curP) 0];
           
rowReduced = rref(constraints);
c1 = rowReduced(1,3);
c2 = rowReduced(2,3);
c = potentialP(1,curP)*X + potentialP(2,curP)*Y + potentialP(3,curP)*Z + potentialP(4,curP);
r = sqrt(u.^2 + v.^2);

% solve system of eqns Ax=b 
A = [(c1*u.*X + c2*Y - Z) -u c.*r.^2];
b = -c;

solns = A\b;
d = solns(1);
p34 = solns(2);
k = solns(3);

% assign p31-p34 to P matrix
P = potentialP(:,curP);
P(end+1) = -c1*d;
P(end+1) = -c2*d;
P(end+1) = d;
P(end+1) = p34;

% get w
syms w
W = vpa(solve(w^2*P(1)^2 + w^2*P(2)^2 + w^2*P(3)^2 - P(9)^2 - P(10)^2 - P(11)^2 == 0),3);
f = 1./W;

% disp('P = ');
% disp(reshape(P,[4,3]).');
% disp('w = ');
% disp(W);
% disp('focal length = ');
% disp(f(2));
% disp('k=');
% disp(k);

POut(:,:,curP) = reshape(P,[4,3])';
end