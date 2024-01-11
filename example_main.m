% Author: Paul Lathrop, pdlathrop@gmail.com
% Date last edited: 1/10/24
%% Description
% main function to initialize environment, pick an initial and final point
% from the largest connected component, run RRT, QRRT, and parallel versions, 
% and summarize and/or plot the result
%% Dependencies:
% Settings.m (class), Dynamics.m (class), createObs.m, RRT.m and RRTpar.m 
% and dependencies, QRRT.m and QRRTpar.m and dependencies, plotfinaltree.m,
% plotobs.m, pointobscheck.m
% genGoodPoint (inline)

clearvars 

max_size = .85; %max obstacle size
num_obs = 525; %number of obstacles

curr_settings = Settings(2,20,0.75,20,37,1000); %settings object
%dimension, config space size, accept solutions within radius, rand points
%constraint, number of rasterizations, oraclebound (to return fault)

sys = Dynamics;
sys.A = [-1.5 -2;1 3];
sys.B = [0.5 .25;0 1];
ploc = [-2.7 -4];
sys.K = place(sys.A,sys.B,ploc);
%oracboundlist = 10:10:200;

k = 1;
while (k <= 50)
    curr_settings.oracbound = 30*rand(); %oracbound changed to be node bound
    loop_param = [k curr_settings.oracbound] 

    obs_list = createObs(curr_settings,num_obs,max_size); %obstaclelist oriented like [obsnumber]->[xlow xhigh ylow yhigh]

    p1 = genGoodPoint(curr_settings,obs_list);
    p2 = genGoodPoint(curr_settings,obs_list);

    tic
    [node_list1,parent_list1,oracle_count1(k)] = RRT(sys,obs_list,p1,p2,curr_settings);
    num_nodes1(k) = length(node_list1(:,1));
    time1(k) = toc;
    1
    %plottree(currsettings.bound,obslist,nodelist1,parentlist1,p2,currsettings.rgoalzone)

    tic
    [node_list2,parent_list2,oracle_count2(k)] = RRTpar(sys,obs_list,p1,p2,curr_settings);
    num_nodes2(k) = length(node_list2(:,1));
    time2(k) = toc;
    2
    %plottree(currsettings.bound,obslist,nodelist2,parentlist2,p2,currsettings.rgoalzone)

    tic
    [node_list3,parent_list3,oracle_count_q(k,:)] = QRRT(sys,obs_list,p1,p2,curr_settings);
    num_nodes3(k) = length(node_list3(:,1));
    time3(k) = toc;
    3
    %plottree(currsettings.bound,obslist,nodelist3,parentlist3,p2,currsettings.rgoalzone)

    tic
    [node_list4,parent_list4,oracle_count_qpar(k,:)] = QRRTpar(sys,obs_list,p1,p2,curr_settings);
    num_nodes4(k) = length(node_list4(:,1));
    time4(k) = toc;
    4

    k = k + 1;
end

oraclecountqsplit = oracle_count_q;
oraclecountqparrsplit = oracle_count_qpar;
for i = 1:length(oracle_count2)
    oracle_count_q(i) = oracle_count_q(i,1) + oracle_count_q(i,2);
    oracle_count_qpar(i) = oracle_count_qpar(i,1) + oracle_count_qpar(i,2);
end
oracle_count_q = oracle_count_q(:,1);
oracle_count_qpar = oracle_count_qpar(:,1);

prop_nodes_to_oracle = [sum(num_nodes1)/sum(oracle_count1) sum(num_nodes2)/sum(oracle_count2) sum(num_nodes3)/sum(oracle_count_q) sum(num_nodes4)/sum(oracle_count_qpar)]


function state = genGoodPoint(current_settings,obs_list)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% generates obstacle free points using obs_list
%% Inputs:
% current_settings: settings class, see Settings.m
% obs_list: double array (size = (n2,4)), rectangular obstacle list
%% Outputs: 
% state: double array (size = (1,2)), obstacle free state
%% Dependencies: 
% pointobscheck.m
%% Uses:
% example_main.m (inline)

goal_good = false; %set false
bound = current_settings.bound;
while(~goal_good) %while goal is not good
    state = [bound*rand() bound*rand()]; %new goal
    goal_good = pointobscheck(obs_list,state); %good if obs free
end
end