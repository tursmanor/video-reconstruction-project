% testing trajectory optimization
clearvars; close all;

%% Constraint cameras
constrP1 = [4 5 6; 5 5 5];
constrF1 = [5 6];
constraint1 = [constrP1(:,3)'; constrF1; constrP1(:,1)'];
constrP2 = [4 4.5 5; 4 4.5 5];
constrF2 = [4 5.5];
constraint2 = [constrP2(:,3)'; constrF2; constrP2(:,1)'];
constrP3 = [4 5 6; 6 5 4];
constrF3 = [6 6];
constraint3 = [constrP3(:,3)'; constrF3; constrP3(:,1)'];
constrP4 = [4 5 6; 4 5 6];
constrF4 = [4 6];
constraint4 = [constrP4(:,3)'; constrF4; constrP4(:,1)'];

makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

%% Testing reproducibility
path4 = [5 8; 8 5];

for i=1:10
    [output,msg4,out4] = pathOpt2D(path4(:,1),path4(:,2),constraint3);
    
    if (msg4.message(1:13) == 'Local minimum')
        disp('path4 pass')
    else
        disp('path4 fail')
    end
    
    testPlotResults(constrP3,constrF3,path4,out4,makeLine);
    
end

%% Valid Setups
path1 = [4 6; 6 6];     % for constraint 1
path2 = [2 8; 2 2];
path3 = [3 8; 5 9];     % for constraint 2
path4 = [5 8; 8 5];     % for constraint 3
path5 = [1 6; 1 3];
path6 = [3 5; 5 10];    % for constraint 4
path7 = [6 8; 2 5];

[~,msg1,out1] = pathOpt2D(path1(:,1),path1(:,2),constraint1);
[~,msg2,out2] = pathOpt2D(path2(:,1),path2(:,2),constraint1);
[~,msg3,out3] = pathOpt2D(path3(:,1),path3(:,2),constraint2);
[~,msg4,out4] = pathOpt2D(path4(:,1),path4(:,2),constraint3);
[~,msg5,out5] = pathOpt2D(path5(:,1),path5(:,2),constraint3);
[~,msg6,out6] = pathOpt2D(path6(:,1),path6(:,2),constraint4);
[~,msg7,out7] = pathOpt2D(path7(:,1),path7(:,2),constraint4);

for i=1:7
    curPath = strcat('path',num2str(i));
    curMsg = eval(strcat('msg',num2str(i)));
    
if (curMsg.message(1:13) == 'Local minimum')
    disp([curPath ' pass'])
else 
    disp([curPath ' fail'])
end

end

testPlotResults(constrP1,constrF1,path1,out1,makeLine);
testPlotResults(constrP1,constrF1,path2,out2,makeLine);
testPlotResults(constrP2,constrF2,path3,out3,makeLine);
testPlotResults(constrP3,constrF3,path4,out4,makeLine);
testPlotResults(constrP3,constrF3,path5,out5,makeLine);
testPlotResults(constrP4,constrF4,path6,out6,makeLine);
testPlotResults(constrP4,constrF4,path7,out7,makeLine);


%% Invalid Setups
path8 = [4 5; 6 8];     % for constraint 1
path9 = [5 8; 8 2];
path10 = [5 5; 7 8];
path11 = [1 8; 8 8];    % for constraint 3
path12 = [8 3; 8 3];
path13 = [3 1; 9 7];    % for constraint 4
path14 = [3 5; 7 2];

[~,msg8,out8] = pathOpt2D(path8(:,1),path8(:,2),constraint1);
[~,msg9,out9] = pathOpt2D(path9(:,1),path9(:,2),constraint1);
[~,msg10,out10] = pathOpt2D(path10(:,1),path10(:,2),constraint1);
[~,msg11,out11] = pathOpt2D(path11(:,1),path11(:,2),constraint3);
[~,msg12,out12] = pathOpt2D(path12(:,1),path12(:,2),constraint3);
[~,msg13,out13] = pathOpt2D(path13(:,1),path13(:,2),constraint4);
[~,msg14,out14] = pathOpt2D(path14(:,1),path14(:,2),constraint4);

for i=8:14
    curPath = strcat('path',num2str(i));
    curMsg = eval(strcat('msg',num2str(i)));
    
if (curMsg.message(1:13) == 'Local minimum')
    disp([curPath ' fail'])
else 
    disp([curPath ' pass'])
end

end

testPlotResults(constrP1,constrF1,path8,out8,makeLine);
testPlotResults(constrP1,constrF1,path9,out9,makeLine);
testPlotResults(constrP1,constrF1,path10,out10,makeLine);
testPlotResults(constrP3,constrF3,path11,out11,makeLine);
testPlotResults(constrP3,constrF3,path12,out12,makeLine);
testPlotResults(constrP4,constrF4,path13,out13,makeLine);
testPlotResults(constrP4,constrF4,path14,out14,makeLine);