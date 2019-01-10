close all; clearvars;

%% Setup
load dataset.mat

n = size(dataset,2);    % num shots
out = struct('cam',0, ...
    'pos',zeros(2,3), ...
    'f',zeros(1,2), ...
    'shots',zeros(n,1));

% first two frames are cams 1 and 2
out(1).cam = 1;
[out(1).pos,out(1).f] = avgCamera(dataset(1).pos,dataset(1).f);
out(1).shots(1) = 1;

out(2).cam = 2;
[out(2).pos,out(2).f] = avgCamera(dataset(2).pos,dataset(2).f);
out(2).shots(2) = 1;

cams = [1,2];   % active cameras in scene
ind = [1,2];    % indices where previous cameras were found (camera 1 was
% last at index 1 of out, etc.)

% for testing
numInfeasible = zeros(2,n);

%% Cluster assignments
for i=3:n
    i
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
            constraint = [constrPos(:,3)'; ...
                         [constrPos(1,2) constrF(2)];...
                          constrPos(:,1)'];
            
            [curOpt,msg] = pathOpt2D(out(ind(j)).pos(:,2)', ...
                                     avgP(:,2)', ...
                                     constraint);
        end
        
        if ((curOpt(1) < opt(1)) && ...
           (sum(msg.message(1:32) == 'Converged to an infeasible point'))/32 ~= 1)
            opt = curOpt;
            bestCam = j;
        end
    
        % for testing
        if ((sum(msg.message(1:32) == 'Converged to an infeasible point'))/32 ~= 1)
            numInfeasible(1,i) = numInfeasible(i) + 1;
        end
    
    end
    
    % check versus some time threshold to determine whether or not to
    % assign shot to an existing camera or a new camera
    thresh = 5;
    if (opt(1) < thresh)
        
        out(i).cam = bestCam;
        ind(bestCam) = i;
    else
        cams = [cams cams(end)+1];
        ind = [ind i];
        out(i).cam = cams(end);   
    end
    
    out(i).pos = avgP;
    out(i).f = avgF;
    out(i).shots(i) = 1;
    
end


