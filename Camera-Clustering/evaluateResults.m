%% Evaluate results
close all; clearvars;

load Experiment1Datasets.mat;
load Experiment1Output_First25Trouble.mat;
extra = load('Experiment1Output-EXTRA.mat');

part2 = load('Experiment1Datasets2.mat');
part3 = load('Experiment1Output_First25Trouble2.mat');

data = [data part2.data];
output = [output part3.output];

diffD1Line = [];
diffD1Traj = [];
diffG1Line = [];
diffG1Traj = [];
diffD2Line = [];
diffD2PWLine = [];
diffD2RRT = [];
diffG2Line = [];
diffG2PWLine = [];
diffG2RRT = [];

D1Line = [];
D1Traj = [];
D2Line = [];
D2PWLine = [];
D2RRT = [];

%for i=1:50
i = 3;
gtSeqV2 = data(i).V2;
gtSeq = horzcat(gtSeqV2.gtCam);

% calculate stright line ground truth cost
gtCost = 0;
for k=1:4
    
   indices = find(gtSeq == k);
   
   if (length(indices) == 1)
       continue;
   else
       for j=1:(length(indices)-1)
           pos1 = gtSeqV2(indices(j)).pos;
           pos2 = gtSeqV2(indices(j+1)).pos;
           gtCost = gtCost + norm(pos2(:,2) - pos1(:,2));
       end
   end
end

% Old methods
costsV2D1Line = output(i).DataV2_DV1_LineCost;
seqsV2D1Line = output(i).DataV2_DV1_LineSeq;
if(isempty(min(costsV2D1Line))) 
    a = -1; 
else
    a = (gtCost-min(costsV2D1Line))/gtCost;
end
diffD1Line = [diffD1Line a];
D1Line = [D1Line sum(costsV2D1Line <= gtCost)];


if (i <= 25)
    costsV2D1Traj = extra.output(i).DataV2_DV1_TrajCost;
    seqsV2D1Traj = extra.output(i).DataV2_DV1_TrajSeq;
else
    costsV2D1Traj = output(i).DataV2_DV1_TrajCost;
    seqsV2D1Traj = output(i).DataV2_DV1_TrajSeq;
end
if(isempty(min(costsV2D1Traj))) 
    a = -1; 
else
    a = (gtCost-min(costsV2D1Traj))/gtCost;
end
diffD1Traj = [diffD1Traj a];
D1Traj = [D1Traj sum(costsV2D1Traj <= gtCost)];

costV2G1Line = output(i).DataV2_GV1_LineCost;
seqV2G1Line = output(i).DataV2_GV1_LineSeq;
if(isempty(min(costV2G1Line)) || min(costV2G1Line) == inf) 
    a = -1; 
else
    a = (gtCost-min(costV2G1Line))/gtCost;
end
diffG1Line = [diffG1Line a];

costV2G1Traj = output(i).DataV2_GV1_TrajCost;
seqV2G1Traj = output(i).DataV2_GV1_TrajSeq;
if(isempty(min(costV2G1Traj)) || min(costV2G1Traj) == inf) 
    a = -1; 
else
    a = (gtCost-min(costV2G1Traj))/gtCost;
end
diffG1Traj = [diffG1Traj a];

% New methods
costsV2D2Line = output(i).DataV2_DV2_LineCost;
seqsV2D2Line = output(i).DataV2_DV2_LineSeq;
if(isempty(min(costsV2D2Line))) 
    a = -1; 
else
    a = (gtCost-min(costsV2D2Line))/gtCost;
end
diffD2Line = [diffD2Line a];
D2Line = [D2Line sum(costsV2D2Line <= gtCost)];

costsV2D2PWLine = output(i).DataV2_DV2_PWLineCost;
seqsV2D2PWLine = output(i).DataV2_DV2_PWLineSeq;
if(isempty(min(costsV2D2PWLine))) 
    a = -1; 
else
    a = (gtCost-min(costsV2D2PWLine))/gtCost;
end
diffD2PWLine = [diffD2PWLine a];
D2PWLine = [D2PWLine sum(costsV2D2PWLine <= gtCost)];

costsV2D2RRT = output(i).DataV2_DV2_RRTCost;
seqsV2D2RRT = output(i).DataV2_DV2_RRTSeq;
if(isempty(min(costsV2D2RRT))) 
    a = -1; 
else
    a = (gtCost-min(costsV2D2RRT))/gtCost;
end
diffD2RRT = [diffD2RRT a];
D2RRT = [D2RRT sum(costsV2D2RRT <= gtCost)];

costV2G2Line = output(i).DataV2_GV2_LineCost;
seqV2G2Line = output(i).DataV2_GV2_LineSeq;
if(isempty(min(costV2G2Line)) ||  min(costV2G2Line) == inf) 
    a = -1; 
else
    a = (gtCost-min(costV2G2Line))/gtCost;
end
diffG2Line = [diffG2Line a];

costV2G2PWLine = output(i).DataV2_GV2_PWLineCost;
seqV2G2PWLine = output(i).DataV2_GV2_PWLineSeq;
if(isempty(min(costV2G2PWLine)) ||  min(costV2G2PWLine) == inf) 
    a = -1; 
else
    a = (gtCost-min(costV2G2PWLine))/gtCost;
end
diffG2PWLine = [diffG2PWLine a];

costV2G2RRT = output(i).DataV2_GV2_RRTCost;
seqV2G2RRT = output(i).DataV2_GV2_RRTSeq;
if(isempty(min(costV2G2RRT)) ||  min(costV2G2RRT) == inf) 
    a = -1; 
else
    a = (gtCost-min(costV2G2RRT))/gtCost;
end
diffG2RRT = [diffG2RRT a];

%end


% mean(diffD1Line), std(diffD1Line)
% mean(diffD1Traj), std(diffD1Traj)
% mean(diffG1Line), std(diffG1Line)
% mean(diffG1Traj), std(diffG1Traj)
% mean(diffD2Line), std(diffD2Line)
% mean(diffD2PWLine), std(diffD2PWLine)
% mean(diffD2RRT), std(diffD2RRT)
% mean(diffG2Line), std(diffG2Line)
% mean(diffG2PWLine), std(diffG2PWLine)
% mean(diffG2RRT), std(diffG2RRT)
% 
% mean(D1Line),std(D1Line)
% mean(D1Traj),std(D1Traj)
% mean(D2Line),std(D2Line)
% mean(D2PWLine),std(D2PWLine)
% mean(D2RRT),std(D2RRT)

%drawResults(costsV2D1Traj,seqsV2D1Traj,costsV2D1Line,seqsV2D1Line,[],[],gtSeq,costV2G1Line,seqV2G1Line,costV2G1Traj,seqV2G1Traj,[],[]);
drawResults(costsV2D2Line,seqsV2D2Line,costsV2D2PWLine,seqsV2D2PWLine,costsV2D2RRT,seqsV2D2RRT,gtSeq,costV2G2Line,seqV2G2Line,costV2G2PWLine,seqV2G2PWLine,costV2G2RRT,seqV2G2RRT);

function [] = drawResults(dynACost,dynASeqs,dynBCost,dynBSeqs,dynCCost,dynCSeqs,gtSeq,greACost,greASeqs,greBCost,greBSeqs,greCCost,greCSeqs)
                                      
% find the dynamic result with the most solutions
[~,index] = max([length(dynASeqs) length(dynBSeqs) length(dynCSeqs)]);
if (index == 1)
    [sortedCosts,sortedCosts2,sortedCosts3,sortedGTInd,gSorted,gSorted2,gSorted3] = alignData(dynASeqs,dynACost,dynBSeqs,dynBCost,dynCSeqs,dynCCost,gtSeq,greACost,greASeqs,greBCost,greBSeqs,greCCost,greCSeqs);
elseif (index == 2)
    [sortedCosts,sortedCosts2,sortedCosts3,sortedGTInd,gSorted,gSorted2,gSorted3] = alignData(dynBSeqs,dynBCost,dynCSeqs,dynCCost,dynASeqs,dynACost,gtSeq,greACost,greASeqs,greBCost,greBSeqs,greCCost,greCSeqs);
else
    [sortedCosts,sortedCosts2,sortedCosts3,sortedGTInd,gSorted,gSorted2,gSorted3] = alignData(dynCSeqs,dynCCost,dynBSeqs,dynBCost,dynASeqs,dynACost,gtSeq, greACost,greASeqs,greBCost,greBSeqs,greCCost,greCSeqs);
end

figure; hold on;
bar(1:size(sortedCosts,2),sortedCosts,'b','FaceAlpha',0.3); hold on;
%bar(find(sortedCosts2),nonzeros(sortedCosts2),'r','FaceAlpha',0.3); hold on;
bar(find(sortedCosts3),nonzeros(sortedCosts3),'r','FaceAlpha',0.3); hold on;
%scatter(gSorted(1),gSorted(2),'x'); hold on;
%scatter(gSorted2(1),gSorted2(2),'x'); hold on;
%scatter(gSorted3(1),gSorted3(2),'x'); hold on;
xline(sortedGTInd,'k','LineWidth',2); hold on;
legend('Dynamic Piecewise Line','Dynamic Line','Ground Truth'); hold on;

title('Distance Error per Sequence');
xlabel('Sequence ID (sorted by ascending cost)');
ylabel('Distance Cost');

figure; hold on;
plot(1:size(sortedCosts,2),sortedCosts); hold on;
plot(find(sortedCosts2),nonzeros(sortedCosts2)); hold on;
plot(find(sortedCosts3),nonzeros(sortedCosts3)); hold on;
scatter(gSorted(1),gSorted(2),'x','r'); hold on;
scatter(gSorted2(1),gSorted2(2),'x','b'); hold on;
scatter(gSorted3(1),gSorted3(2),'x','g'); hold on;
xline(sortedGTInd,'k'); hold on;
legend('Dynamic Piecewise Line','Dynamic RRT','Dynamic Line','Greedy Line','Greedy Piecewise Line','Greedy RRT','Ground Truth'); hold on;

title('Dataset 1: Distance Error per Sequence');
xlabel('Sequence ID (sorted by ascending cost)');
ylabel('Distance Cost');
axis([0 20 8 20]);
end

function [sortedCosts,sortedCosts2,sortedCosts3,sortedGTInd, ...
          gSorted,gSorted2,gSorted3] = alignData(seqs1,cost1,seqs2,cost2,seqs3,cost3,gtSeq, gcost1,gseq1,gcost2,gseq2,gcost3,gseq3)

% sort the largest sequence set
gtInd = find(10 == (sum(seqs1 == gtSeq,2)));
[sortedCosts,ind] = sort(cost1);
sortedSeqs = seqs1(ind,:);
sortedGTInd = find(ind == gtInd);

% sort other lists such that sequences match up
sortedCosts2 = zeros(size(sortedCosts));
sortedCosts3 = zeros(size(sortedCosts));

for i=1:size(seqs2,1)
    curSeq = seqs2(i,:);
    
    for j=1:size(sortedSeqs,1)
        if (isequal(curSeq,sortedSeqs(j,:)))
            sortedCosts2(j) = cost2(i);
            continue;
        end
    end
end

for i=1:size(seqs3,1)
    curSeq = seqs3(i,:);
    for j=1:size(sortedSeqs,1)
        if (isequal(curSeq,sortedSeqs(j,:)))
            sortedCosts3(j) = cost3(i);
            continue;
        end
    end
end

for j=1:size(sortedSeqs,1)
    if (isequal(gseq1,sortedSeqs(j,:)))
        gSorted = [j gcost1];
    end
    if (isequal(gseq2,sortedSeqs(j,:)))
        gSorted2 = [j gcost2];
    end
    if (isequal(gseq3,sortedSeqs(j,:)))
        gSorted3 = [j gcost3];
    end
end

% sanity check: gtInd should be the same for all of them now
%gtInd2 = find(10 == (sum(seqs2 == gtSeq,2)));
%gtInd3 = find(10 == (sum(seqs3 == gtSeq,2)));
%gtCost2 = cost2(gtInd2);
%gtCost3 = cost3(gtInd3);
%sortedGTInd2 = find(sortedCosts2 == gtCost2);
%sortedGTInd3 = find(sortedCosts3 == gtCost3);

end
