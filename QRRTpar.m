function [node_list,parent_list,oracle_count] = QRRTpar(sys,obs_list,x_i,x_goal,current_settings)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% QRRTpar performs a parallel quantum RRT search, from x_i, in a 2D 
% obstacle space grid for a goal location x_goal and returns a tree that 
% includes a node within goal_radius of x_goal. Parallel routine is shared
% database and uses 8 cores
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
oracle_count = 0; %init quantum oracle counter
rand_point_bound = current_settings.randpointbound;
bound = current_settings.bound;
r_goal_zone = current_settings.rgoalzone;
rast_num = current_settings.rastnum;
node_list = x_i; %initialization
parent_list = 1; %parent init
finished = false;

%% Quantum Initializations
n = 8; %number of registers
H = hadamard(n);
D = ia(n); %invert around average matrix
est_iter = 5; %set to 5 here

%% Create Database
while(~finished)
    database = zeros([2^n 3]); %preallocate. 3 for x,y,parentindex (of nodelist)
    for k = 1:2^n
        rand_point = [bound*rand() bound*rand()]; %true random point
        parent_ind = findparent(node_list,rand_point); %find closest parent
        rand_point = drawrandin(node_list(parent_ind,:),rand_point,rand_point_bound); %push randpoint out
        database(k,:) = [rand_point parent_ind]; %add to database
    end
    V_oracle = vf('oracle', n, database,obs_list,node_list,sys,rast_num);

    %% Search Database
    if(length(parent_list)>10), est_iter = 3; end %drop QAA iteration count as tree is growing
    phi_temp = '0'; %create register
    for k = 2:n %fill with 0s
        phi_temp = strcat(phi_temp,'0');
    end
    phi = bin2vec(phi_temp); %goes to vector
    phi = H*phi; %apply Hadamard operator -> superposition

    for i = 1:est_iter
        %Grover's diffusion operator
        phi=V_oracle*phi; %f-conditional 0 inverter
        phi=D*phi; %inverter around mean
        oracle_count = oracle_count + 8; %need to add 8 oracle calls for 8 workers
    end

    %% Parallel routine
    parfor j = 1:8 %only parallelize measurement
        phi_inner(j,:) = measure(phi);
        answer(j) = findanswer(phi_inner(j,:));
        x_pick(j,:) = database(answer(j),1:2);
        xparent(j) = database(answer(j),3);
        %check the sol
        oracle_count = oracle_count + 1;
        final_test(j) = oracle(answer(j)-1,database,obs_list,node_list,sys,rast_num); %get final check results
    end

    %% Process Results
    [x_pick, sort_map, ~] = unique(x_pick,'rows','stable'); %get rid of repeat results, preserve sort_map for parents

    for i = 1:length(x_pick(:,1))
        if(final_test(sort_map(i))) %if final check passes
            node_list = [node_list; x_pick(i,:)];
            parent_list = [parent_list; xparent(sort_map(i))];
            if(quickdist(x_pick,x_goal,2)<=r_goal_zone), finished = true; end %end statement for finding goal
        else
            continue;
        end
    end
end

end