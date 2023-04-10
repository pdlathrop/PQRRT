% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Holds settings of path planning problem
%% Properties
% D: int, dimension (D = 2 generally)
% bound: int, side length of square environment
% rgoalzone: zone around goal where solutions are accepted
% rastnum: number of rasterizations between points in obs checks
% oracbound: value to end functions early on oracle bound limit (unused)
%% Methods
% Settings: setter for Settings object
%% Dependencies:
% N/A
%% Uses:
% many
classdef Settings
   properties
      D 
      bound 
      rgoalzone
      randpointbound
      rastnum
      oracbound
   end
   methods
       function obj = Settings(D,bound,rgoalzone,randpointbound,rastnum,oracbound)
            obj.D = D;
            obj.bound = bound;
            obj.rgoalzone = rgoalzone;
            obj.randpointbound = randpointbound;
            obj.rastnum = rastnum;
            obj.oracbound = oracbound;
       end
   end
end