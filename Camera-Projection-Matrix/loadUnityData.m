function points = loadUnityData(file,type)

fileID = fopen(file,'r');
points = textscan(fileID,'%f','Delimiter',',','Whitespace','()');
fclose(fileID);

numPts = size(points{:},1) / 3;
points = reshape(points{:}',[],numPts)';

if(type == '2D')
    points = points(:,1:2);
end

end