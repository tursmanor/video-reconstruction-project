%%  RESULTS
close all; clearvars;

%% EXPERIMENT 1
% DATA: 
%   n = 10, k = 4
%   time between 30-120 frames (1-4 seconds)
%   100 V1 datasets
%   100 V2 datasets
% METHODS: 
%   greedyV1    line, traj opt
%   greedyV2    line, pw line, RRT
%   dynamicV1   line, traj opt
%   dynamicV2   line, pw line, RRT
% COMBOS:
%   V1 DATA + V1 METHODS (4)
%   V2 DATA + V1 METHODS (4)
%   V2 DATA + V2 METHODS (6)
% SAVE:
%   All generated datasets
%   Output sequences + costs

% DATA
n = 10;
% data = struct('V1',0,'V2',0);
% tic
% for i=1:n
%     data(i).V1 = makeDataset();
%     data(i).V2 = makeDatasetV2();
%     
%     while(isequal(data(i).V2,0))
%         data(i).V2 = makeDatasetV2();
%         disp('redo');
%     end
%     
%     disp(i);
% end
% toc
% save('Experiment1Datasets2','data');
% disp('Data collected');

load Experiment1Datasets;

% COMBOS
output = struct('DataV1_GV1_LineCost',0,'DataV1_GV1_LineSeq',0, ...
    'DataV1_GV1_TrajCost',0,'DataV1_GV1_TrajSeq',0, ...
    'DataV1_DV1_LineCost',0,'DataV1_DV1_LineSeq',0, ...
    'DataV1_DV1_TrajCost',0,'DataV1_DV1_TrajSeq',0, ...
    'DataV2_GV1_LineCost',0,'DataV2_GV1_LineSeq',0, ...
    'DataV2_GV1_TrajCost',0,'DataV2_GV1_TrajSeq',0, ...
    'DataV2_DV1_LineCost',0,'DataV2_DV1_LineSeq',0, ...
    'DataV2_DV1_TrajCost',0,'DataV2_DV1_TrajSeq',0, ...
    'DataV2_GV2_LineCost',0,'DataV2_GV2_LineSeq',0, ...
    'DataV2_GV2_PWLineCost',0,'DataV2_GV2_PWLineSeq',0, ...
    'DataV2_GV2_RRTCost',0,'DataV2_GV2_RRTSeq',0, ...
    'DataV2_DV2_LineCost',0,'DataV2_DV2_LineSeq',0, ...
    'DataV2_DV2_PWLineCost',0,'DataV2_DV2_PWLineSeq',0, ...
    'DataV2_DV2_RRTCost',0,'DataV2_DV2_RRTSeq',0);
% tic
% for i=1:n
%     curDatasetV2 = data(i).V2;
%     
%     % Greedy V1
% %     [out,cost] = greedySoln(curDatasetV1,0);
% %     output(i).DataV1_GV1_LineCost = cost;
% %     output(i).DataV1_GV1_LineSeq = out;
%     
%     [out,cost] = greedySoln(curDatasetV2,0);
%     output(i).DataV2_GV1_LineCost = cost;
%     output(i).DataV2_GV1_LineSeq = out;
%     
% %     [out,cost] = greedySoln(curDatasetV1,1);
% %     output(i).DataV1_GV1_TrajCost = cost;
% %     output(i).DataV1_GV1_TrajSeq = out;
%     
%     [out,cost] = greedySoln(curDatasetV2,1);
%     output(i).DataV2_GV1_TrajCost = cost;
%     output(i).DataV2_GV1_TrajSeq = out;
%     
%     disp('G1 Done');
%     disp(i);
%     close all;
% end
% toc
% save('Experiment1Output2','output');

% tic
% for i=1:n
%     curDatasetV1 = data(i).V1;
%     curDatasetV2 = data(i).V2;
%     
    % Dynamic V1
%     [seqs,costs] = dynamicSoln(curDatasetV1,0);
%     output(i).DataV1_DV1_LineCost = costs;
%     output(i).DataV1_DV1_LineSeq = seqs;
    
%     [seqs,costs] = dynamicSoln(curDatasetV2,0);
%     output(i).DataV2_DV1_LineCost = costs;
%     output(i).DataV2_DV1_LineSeq = seqs;
%     
%     [seqs,costs] = dynamicSoln(curDatasetV1,1);
%     output(i).DataV1_DV1_TrajCost = costs;
%     output(i).DataV1_DV1_TrajSeq = seqs;
%     
%     [seqs,costs] = dynamicSoln(curDatasetV2,1);
%     output(i).DataV2_DV1_TrajCost = costs;
%     output(i).DataV2_DV1_TrajSeq = seqs;
%     
%     disp('D1 Done');
%     disp(i);
%     close all;
% end
% toc
% save('Experiment1Output-EXTRA','output');

% tic
% for i=1:n
%     curDatasetV1 = data(i).V1;
%     curDatasetV2 = data(i).V2;
%     
%     % Greedy V2
%     [out, cost] = greedySolnV2(curDatasetV2,0);
%     output(i).DataV2_GV2_LineCost = cost;
%     output(i).DataV2_GV2_LineSeq = out;
%     
%     disp('G2 Done');
%     disp(i);
%     close all;
% end
% toc
% save('Experiment1Output2','output');
% 
% tic;
% for i=1:n
%     curDatasetV2 = data(i).V2;
%     
%     % Dynamic V2
%     [seqs,costs] = dynamicSolnV2(curDatasetV2,0);
%     %output(i).DataV2_DV2_LineCost = costs;
%     %output(i).DataV2_DV2_LineSeq = seqs;
%     
%     %disp('D2 Done');
%     %disp(i);
%     %close all;
% end
% t = toc
% save('Experiment1Output2','output');
% 
tic
for i=1:n
    curDatasetV2 = data(i).V2;
    
    % Trouble    
%     [out, cost] = greedySolnV2(curDatasetV2,1);
%     output(i).DataV2_GV2_PWLineCost = cost;
%     output(i).DataV2_GV2_PWLineSeq = out;
%     
%     [out, cost] = greedySolnV2(curDatasetV2,2);
%     output(i).DataV2_GV2_RRTCost = cost;
%     output(i).DataV2_GV2_RRTSeq = out;
    
%    [seqs,costs] = dynamicSolnV2(curDatasetV2,1);
%     output(i).DataV2_DV2_PWLineCost = costs;
%     output(i).DataV2_DV2_PWLineSeq = seqs;
%     
     [seqs,costs] = dynamicSolnV2(curDatasetV2,2);
%     output(i).DataV2_DV2_RRTCost = costs;
%     output(i).DataV2_DV2_RRTSeq = seqs;
    
%     disp('D2 Done');
%     disp(i);
%    close all;
end
t = toc
% save('Experiment1Output_First25Trouble2','output');
% 
% disp('Data clustered.');
