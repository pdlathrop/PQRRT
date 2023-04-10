function [] = plotobs(bound,obs_list)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Plots all rectangles of obs_list
%% Inputs: 
% bound: int, bound of environment
% obs_list: double (shape = (num_obs,4)), obstacle environment represented
% as side length locations
%% Outputs:
% N/A
%% Dependencies:
% N/A
%% Uses:
% plottree.m, example_main.m (sometimes)

num_obs = length(obs_list(:,1,1));
figure
hold on
for i = 1:num_obs
    temp_ob = obs_list(i,:);
    rectangle('Position',[temp_ob(1) temp_ob(3) temp_ob(2)-temp_ob(1) temp_ob(4)-temp_ob(3)],'FaceColor',[91/256 91/256 91/256],'EdgeColor',[91/256 91/256 91/256]);
end
xlim([0,bound])
ylim([0,bound])

end