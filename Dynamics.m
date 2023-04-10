% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Holds dynamic system information for ease of passing to functions
%% Properties
% A: double array (size = (2,2)), uncontrolled dynamics of system
% B: double array (size = (2,2)), control matrix of system
% K: double array (size = (1,2)), gain matrix of system
%% Dependencies:
% N/A
%% Uses:
% many
classdef Dynamics
   properties
      A 
      B 
      K
   end
end