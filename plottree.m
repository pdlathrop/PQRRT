function [] = plottree(bound,obs_list,node_list,parent_list,goal,r_goal_zone)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Plots environment, obstacles, goal location, and tree given by node_list 
% and parent_list
%% Inputs: 
% bound: int, bound of environment
% obs_list: double (shape = (num_obs,4)), obstacle environment represented
% as side length locations
% node_list: double array (size = (n,2)), list of current nodes
% parent_list: double array (size = (1,n)), list of current parents
% goal: double array (size = (1,2)), goal state
% r_goal_zone: double, radius of solutions accepted
%% Outputs:
% N/A
%% Dependencies:
% plotobs.m
%% Uses:
% example_main.m (sometimes)

plotobs(bound,obs_list)

plot(node_list(1,1),node_list(1,2),'r.','MarkerSize',44) %plot root node

for i = 2:length(parent_list)
    plot(node_list(i,1),node_list(i,2),'ro')
    line([node_list(i,1) node_list(parent_list(i),1)],[node_list(i,2) node_list(parent_list(i),2)]);
end
plot(goal(1),goal(2),'k*','MarkerSize',18)
viscircles(goal,r_goal_zone);
end