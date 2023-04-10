function [obs_list] = createObs(current_settings,num_obs,max_size)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Function creates a rectangular obstacle environment based on current size
% and number of obstacles
%% Inputs:
% current_settings: Settings Object (see Settings.m)
% num_obs: int, number of rectangular obstacles to place in env
% max_size: double, maximum side length of any 1 obstacle
%% Outputs:
% obs_list: double (shape = (num_obs,4)), obstacle environment represented
% as side length locations [low_x, max_x, low_y, max_y], where rectangular
% obstacle is region between [low_x, max_x] in 1 dimension and [low_y,
% max_y] in 2nd dimension
%% Dependencies:
% Settings.m class
%% Uses:
% example_main.m

min_size  = .25; %no side length smaller than
bound = current_settings.bound;
for i = 1:num_obs 
    low_temp = (bound-max_size)*rand(); low_temp2 = (bound-max_size)*rand();
    obs_list(i,:,:) = [low_temp low_temp+min_size+rand()*(max_size-min_size) low_temp2 low_temp2+min_size+rand()*(max_size-min_size)];
end

end