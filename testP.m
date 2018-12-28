function [d,reproj2D] = testP(P, points2D, points3D)

numP = size(P,3);
numPts = size(points3D,1);
d = zeros(numP,1);

for i=1:numP
    curP = P(:,:,i);
    
    % scale P
    curP = curP ./ max(curP(:));
    
    points2DReprojected = zeros(numPts,3);
    for j=1:numPts
        cur2DPt = curP * [points3D(j,:)'; 1];
        points2DReprojected(j,:) = cur2DPt ./ cur2DPt(3); % norm so third entry is one
    end
    
    points2DReprojected = points2DReprojected(:,1:2);
    
    % Test L2 dist
    for j=1:numPts
        curD = sqrt(sum((points2D(j,:) - points2DReprojected(j,:)).^2));
        d(i) = d(i) + curD;
    end
   
    d(i) = d(i) / numPts;
    
    reproj2D = points2DReprojected;
    
end

end