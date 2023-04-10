function b = oracle(i,database,obs_list,node_list,system,rast_num)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Function takes point and parent of index i of database and tests whether
% a dynamic path (using system) from the parent to the point is outside of 
% obstacles in obs_list according to rasterization number rast_num
% Mostly just invoking dynamicscheck.m 
%% Inputs:
% i: int, index of database to check
% database: double array (size (n,3)), database of possible points and
% parents
% obs_list: double array (size = (n2,4)), rectangular obstacle list
% node_list: double array (size = (n3,2)), list of current nodes so parent
% can be identified
% system: dynamic system to be used for reachability, see Dynamics.m
% rast_num: number of rasterizations between subsequent points of path
%% Outputs:
% b: Boolean, 1 for reachable and 0 else
%% Dependencies:
% dynamicscheck.m and dependencies
%% Uses:
% QRRT.m, QRRTpar.m

i = i+1; %1 is added due to for loop in vf.m 0:N-1
raw_data = database(i,:);
point = raw_data(1:2);
parent_ind = raw_data(3);

b = dynamicscheck(system,obs_list,node_list(parent_ind,:),point,rast_num);
end