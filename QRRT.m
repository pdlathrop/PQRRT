function [node_list,parent_list,oracle_count] = QRRT(sys,obs_list,x_i,x_goal,current_settings)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% QRRT performs a quantum RRT search, from x_i, in a 2D obstacle space grid
% for a goal location x_goal and returns a tree that includes a node within
% goal_radius of x_goal
% Notes: size of database and number of QAA iterations are up to user
% preference and specific environment
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
% hadamard.m, ia.m, vf.m, bin2vec.m, dec2vec.m, cf_assert.m, cf_approx.m, measure.m, findanswer.m,
% oracle.m, dynamicscheck.m, pointobscheck.m, findParent.m, quickdist.m
%% Uses:
% main.m

%% Initializations
oracle_count = 0; 
rand_point_bound = current_settings.randpointbound;
bound = current_settings.bound;
r_goal_zone = current_settings.rgoalzone;
rast_num = current_settings.rastnum;
node_list = x_i; 
parent_list = 1; 
finished = false;

%% Quantum Initializations
n = 8; %num registers
H = hadamard(n);
D = ia(n); %invert around average matrix
estiter = 5; %set to 5 here

while(~finished)
    %% Create Database
    database = zeros([2^n 3]); %preallocate. 3 for x,y,parentindex (of nodelist)
    for k = 1:2^n
        rand_point = [bound*rand() bound*rand()]; %true random point
        %end
        parent_ind = findparent(node_list,rand_point); %find closest parent
        rand_point = drawrandin(node_list(parent_ind,:),rand_point,rand_point_bound); %push randpoint out
        database(k,:) = [rand_point parent_ind]; %add to database
    end
    V_oracle = vf('oracle', n, database,obs_list,node_list,sys,rast_num);

    %% Search Database
    phi_temp = '0'; %create register
    for k = 2:n %fill with 0s
        phi_temp = strcat(phi_temp,'0');
    end
    phi = bin2vec(phi_temp); %goes to vector
    phi = H*phi; %apply Hadamard operator -> superposition

    if(length(parent_list)>10), estiter = 3; end
    for i = 1:estiter
        %Grover's diffusion operator
        phi=V_oracle*phi; %f-conditional 0 inverter
        phi=D*phi; %inverter around mean
        oracle_count = oracle_count + 1;
    end
    %% Measure and add
    phi = measure(phi);
    answer = findanswer(phi);
    x_pick = database(answer,1:2);
    x_parent = database(answer,3);
    oracle_count = oracle_count + 1;
    if(oracle(answer-1,database,obs_list,node_list,sys,rast_num)) %answer-1 due to +1 offset in oracle.m
        node_list = [node_list;x_pick];
        parent_list = [parent_list;x_parent];
        if(quickdist(x_pick,x_goal,2)<=r_goal_zone), finished = true; end %end statement for finding goal
    else
        continue;
    end
end

end