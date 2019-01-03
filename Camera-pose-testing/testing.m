% tmp testing
clearvars;

% problem child for i=5 for main.m
constraints =  [4.5798    8.8901;
                4.0195    8.5956;
                3.4592    7.2336];
initPos = [5.8642    2.9363];
endPos = [3.1584    4.6423];

[curOpt,msg] = pathOpt2D(initPos,endPos,constraints);

% figure;
sceneSize = [0 10 0 10];
makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;
% drawCamera(constraints',makeLine,sceneSize,[4.0195    8.5956])
% 

%% try 2
% x = constraints(3,1);
% x1 = constraints(2,1);
% y1 = constraints(2,2);
% x2 = constraints(1,1);
% y2 = constraints(1,2);
% 
% X = constraints(1,1);
% X2 = constraints(3,1);
% Y2 = constraints(3,2);
% 
% figure;
% scatter(x,makeLine(x,x1,y1,x2,y2)); hold on;
% scatter(X,makeLine(X,x1,y1,X2,Y2)); hold on;
% drawCamera(constraints',makeLine,sceneSize,[4.0195    8.5956]);
% 
% % try 2
% constraints2 = [x,makeLine(x,x1,y1,x2,y2);
%                x1 y1;
%                X,makeLine(X,x1,y1,X2,Y2)];
%            
% [curOpt,msg] = pathOpt2D(initPos,endPos,constraints2);

