%% RQ decomposition
% source: https://ksimek.github.io/2012/08/14/decompose/
function [R Q] = rq(M)
[Q,R] = qr(flipud(M)');
R = flipud(R');
R = fliplr(R);

Q = Q';
Q = flipud(Q);
end