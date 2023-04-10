function parent_ind = findparent(node_list,rand_point)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Inline function that finds the nearest parent in node_list (based on Manhattan norm) of x
%% Inputs:
% rand_point: double array (shape = (1,2)), query point
% node_list: double array (shape = (n,2)), list of possible parents
%% Outputs:
% parent_ind: int, index of nearest parent to x
%% Dependencies:
% quick1Norm.m
%% Uses:
% QRRT.m, RRT.m, QRRTpar.m, RRTpar.m
min_len = inf;
for i = 1:length(node_list(:,1))
    temp_dist = quickdist(rand_point,node_list(i,:),2);
    if(temp_dist < min_len), min_len = temp_dist; parent_ind = i; end
end
end