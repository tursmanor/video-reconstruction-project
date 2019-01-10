function [avgPos, avgF] = avgCamera(pos, f)
% calculates the average position and focal length of a camera in a shot
% from the dataset

avgPos = zeros(2,3);
for i=1:2:size(pos,1)
    avgPos = avgPos + pos(i:i+1,:);
end
avgPos = avgPos / (size(pos,1)/2);

avgF = zeros(1,2);
for i=1:size(f,1)
    avgF = avgF + f(i,:);
end
avgF = avgF / size(f,1);

end