%% Draw comparison plots
close all; clearvars;

%% Handle dynamic data
load DynamicTraj.mat;

[sortedCosts,ind] = sort(costs);
sortedSeqs = seqs(ind,:);
sortedGTInd = find(ind == gtInd);

load DynamicLine.mat;

sortedCosts2 = zeros(size(sortedCosts));
for i=1:size(seqs,1)

    curSeq = seqs(i,:);   
    for j=1:size(sortedSeqs,1)
        if (isequal(curSeq,sortedSeqs(j,:)))
           sortedCosts2(j) = costs(i);
           continue;
        end
    end
end

gtCost2 = costs(gtInd);
sortedGTInd2 = find(sortedCosts2 == gtCost2);

%% Handle greedy data


%% Plot results
figure;
line(1:size(sortedCosts,2),sortedCosts);
hold on;
scatter(sortedGTInd,sortedCosts(sortedGTInd));
hold on;
bar(find(sortedCosts2),nonzeros(sortedCosts2),'r','FaceAlpha',0.5);
hold on;
legend('Dynamic Traj Opt','GT','Dynamic Line');
title('Comparison Plots');
xlabel('Sequences');
ylabel('Distance Cost Error');
%axis([0 100 0 15]);