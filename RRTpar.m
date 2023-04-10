function [node_list,parent_list,oracle_count] = RRTpar(sys,obs_list,x_i,x_goal,current_settings)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% RRTpar performs a parallel RRT search, from x_i, in a 2D obstacle space 
% grid for a goal location x_goal and returns a tree that includes a node 
% within goal_radius of x_goal. Parallel computation adds nodes from 8
% workers before checking end condition
%% Inputs: 
% sys: dynamics class, see Dynamics.m
% obs_list: double (shape = (num_obs,4)), obstacle environment represented
% as side length locations
% x_i: double (shape = (1,2)), initial robot state, forming root of tree
% x_goal: double (shape = (1,2)), goal robot state
% current_settings: settings class, see Settings.m
%% Outputs:
% node_list: double array (shape = (n,2)), list of nodes of output tree
% parent_list: double array (shape = (1,n)), list of parents of each node of the tree
% oracle_count: int, total number of oracle calls
%% Dependencies:
% Settings.m (class), Dynamics.m (class)
% dynamicscheck.m, pointobscheck.m, findparent.m, quickdist.m
%% Uses:
% main.m

%% Initializations
oracle_count = 0;
randpointbound = current_settings.randpointbound;
bound = current_settings.bound;
rgoalzone = current_settings.rgoalzone;
rastnum = current_settings.rastnum;
node_list = x_i; %initialization
parent_list = 1; %parent init
finished = false;

while(~finished)
    %% Parallel routine
    parfor i = 1:8
        rand_point(i,:) = [bound*rand() bound*rand()]; %true random point
        parent_ind(i) = findparent(node_list,rand_point(i,:)); %find closest parent
        rand_point(i,:) = drawrandin(node_list(parent_ind(i),:),rand_point(i,:),randpointbound); %bring random point in
        free_test(i) = dynamicscheck(sys,obs_list,node_list(parent_ind(i),:),rand_point(i,:),rastnum); %dynamics based test
        oracle_count = oracle_count + 1;
    end
    %% Process results
    for i = 1:8
        if(free_test(i))
            node_list = [node_list;rand_point(i,:)]; %add to nodelist
            parent_list = [parent_list;parent_ind(i)]; %add to parentlist
        else
            continue;
        end
        if(quickdist(rand_point(i,:),x_goal,2)<=rgoalzone), finished = true; break; end
    end
end

end