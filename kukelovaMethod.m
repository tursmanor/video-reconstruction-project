function [intrinsics,PMat] = kukelovaMethod(points2D,points3D)

% Calculate projection matrix
PMat = pnp5(points2D,points3D); %.* 100; % scaling factor

intrinsics = struct('K',zeros(3,3),'R',zeros(3,3),'camCenterW',zeros(1,3));

for i=1:size(PMat,3)

    P = PMat(:,:,i);
    intrinsics(i).camCenterW = -inv(P(1:3,1:3)) * P(:,4);
    
    M = P(1:3,1:3);
    [K,R] = rq(M);
    T = diag(sign(diag(K)));
    K = K * T;
    R = T * R;
    
    % fix scale
    scale = 1/K(3,3);
    K = K .* scale;
    
    intrinsics(i).K = K;
    intrinsics(i).R = R;

end

