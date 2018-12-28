function [pt,ptL,ptR,slopeL,slopeR,lLine,rLine] = makeLines(camConst)

pt = camConst(2,:);
ptL = camConst(1,:);
ptR = camConst(3,:);
slopeL = (ptL(2) - pt(2)) / (ptL(1) - pt(1));
slopeR = (ptR(2) - pt(2)) / (ptR(1) - pt(1));
lLine = @(x) slopeL * (x - pt(1)) + pt(2);
rLine = @(x) slopeR * (x - pt(1)) + pt(2);

end

