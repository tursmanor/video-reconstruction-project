%%  RESULTS
close all; clearvars;

%% EXPERIMENT 1
% 10 shots, 4 cameras
% time between 30-150 frames (1-5 seconds)
% 100 trials per experiment
% Metrics: cam assignment, num cameras, total distance

n = 1;
datasets = struct('GT',0,'Algo',0);

% data collection
for i=1:n
    datasets(i).GT = makeDataset([15 15],4);
    datasets(i).Algo = main(datasets(i).GT); 
    i
end

save('round2-experiment1','datasets');

% analysis
% load experiment1.mat
% 
% n = 100;
% acc = 0;
% diff = 0;
% sameCamNum = 0;
% hist = [];
% for i=1:n
%     
%     algoOut = vertcat(datasets(i).Algo.cam);
%     gtOut = vertcat(datasets(i).GT.gtCam); 
%     accuracy = sum(algoOut == gtOut) / 10;
%     acc = acc + accuracy;
%     
%     k1 = max(vertcat(datasets(i).GT.gtCam));
%     k2 = max(vertcat(datasets(i).Algo.cam));    
%     sameCamNum = sameCamNum + abs((k2 - k1) / k1);
%     
%     d = distanceComparison(datasets(i).GT, datasets(i).Algo);
%     if (d(1) ~= 0)
%         diff = diff + abs(((d(2)-d(1)) / d(1)));
%     end
%     hist = [hist d(1)-d(2)];
% end
% 
% acc / n
% diff / n
% sameCamNum / n
% histogram(hist,50); hold on;
% ylabel('count');
% xlabel('GT Distance - Algo Distance');
% title('Experiment One Results');

%% EXPERIMENT 2
% 10 shots, 4 cameras
% time from 15 to 150 frames in 15 frame increments
% 100 trials per experiment
% Metrics: cam assignment, num cameras, total distance

%n = 20;
% datasets = struct('GT',0,'Algo',0);
%frameInc = 15:15:150;
% frameInc = 165:15:300;

% data collection
% for j=frameInc
%     for i=1:n
%         datasets(i).GT = makeDataset([j j],4);
%         datasets(i).Algo = main(datasets(i).GT);
%         i
%     end
%     save(['experiment2-' num2str(j)],'datasets');
%     j
% end

% analysis
% load experiment2-300.mat
% 
% n = 20;
% acc = 0;
% diff = 0;
% sameCamNum = 0;
% hist = [];
% for i=1:n
%     
%     algoOut = vertcat(datasets(i).Algo.cam);
%     gtOut = vertcat(datasets(i).GT.gtCam); 
%     accuracy = sum(algoOut == gtOut) / 10;
%     acc = acc + accuracy;
%     
%     k1 = max(vertcat(datasets(i).GT.gtCam));
%     k2 = max(vertcat(datasets(i).Algo.cam));    
%     sameCamNum = sameCamNum + abs((k2 - k1) / k1);
%     
%     d = distanceComparison(datasets(i).GT, datasets(i).Algo);
%     if (d(1) ~= 0)
%         diff = diff + abs(((d(2)-d(1)) / d(1)));
%     end
%     hist = [hist d(1)-d(2)];
% end
% 
% acc / n
% sameCamNum / n
% diff / n
% histogram(hist,50); hold on;
% ylabel('count');
% xlabel('GT Distance - Algo Distance');


%% EXPERIMENT 3
% 10 shots, 4 cameras
% testing consistency
% 10 trials, 10 runs per trial
% Metric: get same output every time

% n = 10;
% out = [];
% % data collection
% for i=1:n
%     same = 0;
%     data= makeDataset([30 30],4);
%     prev = main(data);
%     for j=1:4
%         cur = main(data);
%        if all(vertcat(cur.cam) == vertcat(prev.cam))
%            same = same + 1; 
%        end
%         prev = cur;
%     end
%     out = [out same/10];
% end

%% EXPERIMENT 4
% noise testing
% keep f noise fixed, r/4 as base
% increments: r/4, r/3.5, r/3, r/2.5, r/2, r/1.5, r
% 
% n = 20;
% datasets = struct('GT',0,'Algo',0);
% %denominators = 4:-.5:1;
% denominators = 0.0156;
% 
% for j=denominators
%     for i=1:n
%         datasets(i).GT = makeDataset([30 150],j);
%         datasets(i).Algo = main(datasets(i).GT);
%         i
%     end
%     save(['experiment4-' num2str(j)],'datasets');
%     j
% end

% analysis
% load experiment4-0-0625.mat
% 
% n = 20;
% acc = 0;
% diff = 0;
% sameCamNum = 0;
% hist = [];
% for i=1:n
%     
%     algoOut = vertcat(datasets(i).Algo.cam);
%     gtOut = vertcat(datasets(i).GT.gtCam); 
%     accuracy = sum(algoOut == gtOut) / 10;
%     acc = acc + accuracy;
%     
%     k1 = max(vertcat(datasets(i).GT.gtCam));
%     k2 = max(vertcat(datasets(i).Algo.cam));    
%     sameCamNum = sameCamNum + abs((k2 - k1) / k1);
%     
%     d = distanceComparison(datasets(i).GT, datasets(i).Algo);
%     if (d(1) ~= 0)
%         diff = diff + abs(((d(2)-d(1)) / d(1)));
%     end
%     hist = [hist d(1)-d(2)];
% end
% 
% acc / n
% diff / n
% sameCamNum / n
% histogram(hist,50); hold on;
% ylabel('count');
% xlabel('GT Distance - Algo Distance');
% title('Experiment One Results');

%% Plotting stuff for experiment 4
% xdata = [4,3.5,3,2.5,2,1.5,1,0.25,0.0625];
% acc = [0.8950,0.90,0.92,0.95,0.9050,0.83,0.8550,0.7750,0.4100];
% distErr = [0.2554,0.2166,0.0994,0.0856,0.0976,0.1786,0.0950,0.3666,0.7495];
% camNum = [0.05,0.0417,0.0250,0.0625,0.0667,0.1792,0.1375,0.3458,1.0083];
% 
% figure;
% plot(xdata,acc); hold on;
% xlabel('Frames per Shot');
% ylabel('Clustering Accuracy');
% title('Clustering Accuracy v Frame Count');
% 
% figure;
% plot(xdata,camNum); hold on;
% xlabel('Frames per Shot');
% ylabel('Camera Number Percent Error');
% title('Camera Number Percent Error v Frame Count');
% 
% figure;
% plot(xdata,distErr); hold on;
% xlabel('Frames per Shot');
% ylabel('Distance Percent Error');
% title('Distance Percent Error v Frame Count');

%% Plotting stuff for experiment 2

% xdata = 15:15:300;
% accuracies = [1,1,1,1,0.995,1,0.975,0.945,0.87,0.86,0.8350,0.6750,0.7550,0.5500,0.6050,0.5350,0.56,0.59,0.58,0.5450];
% % camNum = [1,1,1,1,1,1,1,0.9,0.85,0.75,0.7,0.65,0.55,0.45,0.3,0.45,0.25,0.35,0.25,0.1];
% camNum =     [0,0,0,0,0,0,0,0.0292,0.0375,0.0625,0.0750,0.0958,0.1292,0.1542,0.2125,0.1458,0.2,0.1750,0.2292,0.3083];
% distErr =    [0,0,0,0,0.0019,0,0.0129,0.0852,0.1473,0.1519,0.2598,0.1973,0.2068,0.5198,0.6777,0.6280,0.7556,0.6776,1.8714,2.3070];
% 
% figure;
% plot(xdata,accuracies); hold on;
% xlabel('Frames per Shot');
% ylabel('Clustering Accuracy');
% title('Clustering Accuracy v Frame Count');
% 
% figure;
% plot(xdata,camNum); hold on;
% xlabel('Frames per Shot');
% ylabel('Camera Number Percent Error');
% title('Camera Number Percent Error v Frame Count');
% 
% figure;
% plot(xdata,distErr); hold on;
% xlabel('Frames per Shot');
% ylabel('Distance Percent Error');
% title('Distance Percent Error v Frame Count');