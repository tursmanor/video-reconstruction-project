function  distDiff = distanceComparison(dataset, algo)
% compare distance traveled between GT and algo output
n = size(dataset,2);
distGT = struct('pos',zeros(1,2));
distAlgo = struct('pos',zeros(1,2));

k = max(max(vertcat(dataset.gtCam)),max(vertcat(algo.cam)));

for i=1:k
    distGT(i).pos = zeros(1,2);
    distAlgo(i).pos = zeros(1,2);
end

for i=1:n

    curCam = dataset(i).gtCam;
    [curPos,~] = avgCamera(dataset(i).pos,dataset(i).f);
    curP = curPos(:,2)';
    distGT(curCam).pos = [distGT(curCam).pos;curP];   
    
    curCam = algo(i).cam;
    curP = algo(i).pos(:,2)';
    distAlgo(curCam).pos = [distAlgo(curCam).pos;curP];   
    
end

distanceOut = 0;
for i=1:k
   
    curRow = distGT(i).pos;
    curRow = curRow(2:end,:);
    
    rowDist = 0;
    for j=1:size(curRow,1)-1
        rowDist = rowDist + norm(curRow(j+1) - curRow(j));
    end
    
    distanceOut = distanceOut + rowDist;
    
end

distanceAlgo = 0;
for i=1:k
   
    curRow = distAlgo(i).pos;
    curRow = curRow(2:end,:);
    
    rowDist = 0;
    for j=1:size(curRow,1)-1
        rowDist = rowDist + norm(curRow(j+1) - curRow(j));
    end
    
    distanceAlgo = distanceAlgo + rowDist;
    
end


distDiff = [distanceOut distanceAlgo];

end