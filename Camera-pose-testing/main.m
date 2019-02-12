function out = main(dataset)
%% Setup
%load dataset-radius-test.mat
makeLine =@(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

n = size(dataset,2);    % num shots
out = struct('cam',0, ...
    'pos',zeros(2,3), ...
    'f',zeros(1,2));

% first two frames are cams 1 and 2
out(1).cam = 1;
[out(1).pos,out(1).f] = avgCamera(dataset(1).pos,dataset(1).f);

out(2).cam = 2;
[out(2).pos,out(2).f] = avgCamera(dataset(2).pos,dataset(2).f);

cams = [1,2];   % active cameras in scene
ind = [1,2];    % indices where previous cameras were found (camera 1 was
% last at index 1 of out, etc.)

% for testing
numInfeasible = zeros(2,n);

%% Cluster assignments
for i=3:n
    prevCam = out(i-1).cam;
    
    % get average position and focal length for camera
    curData = dataset(i);
    [avgP,avgF] = avgCamera(curData.pos,curData.f);
    
    % testing
    numInfeasible(2,i) = size(cams,2);
    
    % move all current cameras to new position
    opt = inf;
    for j=cams
        
        if (j == prevCam)
            continue;
        else
            
            constrPos = out(ind(prevCam)).pos;
            constrF = out(ind(prevCam)).f;
            constraint = [constrPos(:,3)'; constrF; constrPos(:,1)'];
                          
            [curOpt,msg,posOut] = pathOpt2D(out(ind(j)).pos(:,2)', ...
                                     avgP(:,2)', ...
                                     constraint);
                                                                                  
            % testing
%             figure;
%             [~] = drawCamera(out(ind(j)).pos,makeLine,[0 10 0 10],out(ind(j)).f,'red'); hold on;
%             [~] = drawCamera(constrPos,makeLine,[0 10 0 10],constrF,'blue'); hold on;
%             plot(posOut(:,1),posOut(:,2)); hold on;
%             axis([0 10 0 10]);
            
        end
  
        if (curOpt(1) < opt(1)) & (msg.message(1:5) == 'Local')
            opt = curOpt;
            bestCam = j;
        end
    
        % for testing
        if (msg.message(1:32) == 'Converged to an infeasible point')
            numInfeasible(1,i) = numInfeasible(i) + 1;
        end
        
        disp('current best time');
        opt(1)
    
    end
    
%     if (norm(out(ind(bestCam)).pos(:,2)' - avgP(:,2)') > 0.0937)
%         disp('BAD');
%     end
    
    % check versus some time threshold to determine whether or not to
    % assign shot to an existing camera or a new camera
    thresh = (dataset(i-1).frame / 30)% * 1.5;
    if (opt(1) <= thresh)
        out(i).cam = bestCam;
        ind(bestCam) = i;
    else
        cams = [cams cams(end)+1];
        ind = [ind i];
        out(i).cam = cams(end);   
    end
    
    out(i).pos = avgP;
    out(i).f = avgF;
    
end

%save('clustering-radius-test','out');

%% Quantify results
algoOut = vertcat(out.cam);
gtOut = vertcat(dataset.gtCam); 
accuracy = sum(algoOut == gtOut) / n

end