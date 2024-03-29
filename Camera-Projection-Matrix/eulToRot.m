function R = eulToRot(eulerAngles)
% assuming order z,x,y
% assumes angles in degrees bc unity output is ungodly

thetaX = eulerAngles(1);
thetaY = eulerAngles(2);
thetaZ = eulerAngles(3);

Rx = [1 0 0; 0 cosd(thetaX) -sind(thetaX); 0 sind(thetaX) cosd(thetaX)];
Ry = [cosd(thetaY) 0 sind(thetaY); 0 1 0; -sind(thetaY) 0 cosd(thetaY)];
Rz = [cosd(thetaZ) -sind(thetaZ) 0; sind(thetaZ) cosd(thetaZ) 0; 0 0 1];

R = Rz * Rx * Ry;

end