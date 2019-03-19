%% Get P
% Set up linear system of eqns
%
%[ X1 Y1 Z1 1 0  0  0  0 -u1*X1 -u1*Y1 -u1*Z1 -u1        [M11
%  0  0  0  0 X1 Y1 Z1 1 -v1*X1 -v1*Y1 -v1*Z1 -v1         .
%  .  .  .  . .  .  .  .    .     .      .     .     *    .   = 0 matrix
%  Xn Yn Zn 1 0  0  0  0 -un*Xn -un*Yn -un*Zn -un         .
%  0  0  0  0 Xn Yn Zn 1 -vn*Xn -vn*Yn -vn*Zn -vn]        M34]

function P = getProjection(points2D,points3D)

n = size(points2D,1);
data_matrix = zeros(2*n,12);

for i=1:n
    X = points3D(i,1);
    Y = points3D(i,2);
    Z = points3D(i,3);
    u = points2D(i,1);
    v = points2D(i,2);
    
    data_matrix((2*i) - 1,:) = [X Y Z 1 ...
                                0 0 0 0 ...
                               -u*X -u*Y -u*Z -u];
    data_matrix((2*i),:) = [0 0 0 0 ...
                            X Y Z 1 ...
                           -v*X -v*Y -v*Z -v];
end

% Solve for entries of M using SVD
% first, find smallest singular value along diag of S
% take corresp col of V-- this is M
[~,S,V] = svd(data_matrix);
[~,indx] = min(diag(S));
P = V(:,indx);

% Reshape to proj. matrix
% have to do some tricky transpose business, because reshape takes elements
% column-wise instead of row-wise
P = reshape(P, [4 3])';
end