function [intrinsics] = textbookMethod(points2D,points3D)

% Calculate projection matrix
P = getProjection(points2D,points3D);

% Get estimates of K
% following p163 of the multiple view geometry book
% 150 in 2000 ed
%C w a tilda is the coord of the cam center in the world coord frame

intrinsics = struct('K', zeros(3,3), ...
                    'R', zeros(3,3), ...
                    'camCenterW', zeros(1,3));


camCenterWorld = -inv(P(1:3,1:3)) * P(:,4);
    
% P = [M | -MC] where C is the world coord of the camera center
% M is left 3x3 submatrix of P
M = P(1:3,1:3);
    
% to get K and R, use RQ decomposition
[K,R] = rq(M);
    
% make diagonal of K positive
% source: https://ksimek.github.io/2012/08/14/decompose/
T = diag(sign(diag(K)));
K = K * T;
R = T * R;
    
% fix scale
scale = 1/K(3,3);
K = K .* scale;
    
intrinsics.K = K;
intrinsics.camCenterW = camCenterWorld;
intrinsics.R = R;

end

