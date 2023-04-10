function point_check = pointobscheck(obslist,point)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Function checks if point is inside any obstacle of obslist, and returns
% true if point is obstacle free anf false otherwise
%% Inputs:
% obslist: double array (size = (n,4)), rectangular obstacle list
% point: double array (size = (1,2)), point to query
%% Outputs:
% point_check: Boolean, 1 if obstacle free, 0 otherwise
%% Dependencies:
% N/A
%% Uses:
% example_main.m
tol = 0;
num_obs = length(obslist(:,1,1));
point_check = true;
for i = 1:num_obs
    temp_check = ~((point(1)>=obslist(i,1)-tol) && (point(1)<=obslist(i,2)+tol) && (point(2)>=obslist(i,3)-tol) && (point(2)<=obslist(i,4)+tol));
    point_check = point_check && temp_check;
    if(~temp_check), break; end %efficiency line: if one is bad, exit early
end

end
