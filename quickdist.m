function dist = quickdist(p1,p2,d)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Finds distance of Ld norm between p1 and p2
%% Inputs:
% p1: double array (shape = (1,2)), query point 1
% np2: double array (shape = (1,2)), query point 2
%% Outputs:
% dist: double, distance in L-d norm from p1 to p2
%% Dependencies:
% N/A
%% Uses:
% QRRT.m, QRRTpar.m, RRT.m, RRTpar.m
if(d == 1)
    dist = abs(p1(1)-p2(1)) + abs(p1(2) - p2(2));
    return
elseif(d == 2)
    dist = sqrt((p2(1)-p1(1))^2+(p2(2)-p1(2))^2);
    return
end
end