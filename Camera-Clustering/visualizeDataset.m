%% Visualize dataset
% draw the four camera positions, color code them, and have them move frame
% by frame

close all; clearvars;
load 'Results/dataset-test2.mat'
%dataset = datasets(46).GT;
sceneSize = [-10 15 -10 15];
makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;
filename = 'Results/dataset-test2.gif';

fig = figure(1);
axis([0 10 0 10]); hold on;
camHandles = zeros(4,5);
prevCam = 0;

for i = 1:size(dataset,2)
    
    [avgP,avgF] = avgCamera(dataset(i).pos,dataset(i).f);
    curCam = dataset(i).gtCam;
    
    % set distinct colors for each camera, assuming 4 cameras
    if (curCam == 1)
        color = 'blue';
    elseif (curCam == 2)
        color = 'red';
    elseif (curCam == 3)
        color = 'green';
    else
        color = 'magenta';
    end
    
    % erase camera's previous location
    if (camHandles(curCam,:) ~= 0)
        set(camHandles(curCam,:),'Visible','off');
    end
    
    % draw camera
    [figHandles] = drawCamera(avgP,makeLine,sceneSize,avgF,color);
    camHandles(curCam,:) = figHandles;
    
    % fill in fov for previous camera
    if prevCam ~= 0
        set(camHandles(prevCam,end),'Visible','on');
        cameras = ones(1,4);
        cameras(prevCam) = 0;
        nonActiveCams = camHandles(logical(cameras),end);
        noZeros = nonActiveCams(nonActiveCams ~= 0);
        set(noZeros,'Visible','off');
    else
        set(camHandles(curCam,end),'Visible','on');
    end
    
    prevCam = curCam;
    
    % get time of shot
    time = dataset(i).frame / 30;
    
    % save as a gif
    im = frame2im(getframe(fig));
    [A,map] = rgb2ind(im,256);
    if (i == 1)
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',time);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',time);
    end
    
end