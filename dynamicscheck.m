function test = dynamicscheck(system,obs_list,point1,point2,rast_num)
% Author: Paul Lathrop, MAE, UCSD
% Date last edited: 4/9/23
%% Description:
% Reachability test:
% Function tests whether a dynamic path (using system) from point1 to point2 
% is outside of  obstacles in obs_list according to rasterization number 
% rast_num
%% Inputs:
% system: dynamic system to be used for reachability, see Dynamics.m
% obs_list: double array (size = (n2,4)), rectangular obstacle list
% point1: double array (size = (1,2)), origin point for reachability
% point2: double array (size = (1,2)), final point 
% rast_num: number of rasterizations between subsequent points of path
%% Outputs:
% test: Boolean, 1 if provably reachable (obstacle free), 0 otherwise
%% Dependencies:
% Dynamics.m, pointobscheck.m
% quickrast (inline), sys (inline)
%% Uses:
% oracle.m, RRT.m, RRTpar.m

test = true;

A = system.A;
B = system.B;
K = system.K;
t_span = [0:.05 5]; %time span
init_e = point1-point2; %error term
[t,y] = ode45(@sys,t_span,init_e); %numerical integration of controller and dynamics
y = y + point2; %offset output back from error to state

for i = 1:min(length(t),50) %for every one of the output points (except for late ones)
    if(i>1 && i<=25) %first couple points quite spread
        temp_good = quickrast(y(i,:),y(i-1,:),rast_num,obs_list);
    else
        temp_good = pointobscheck(obs_list,y(i,:));
    end
    test = test && temp_good;
    if(~test), break; end %efficiency exit early
end

    function  dx = sys(~,x)
        u = -K*(x);
        dx = A*(x) + B*u;
    end
    function supra_test = quickrast(p1,p2,rast_num,obs_list)
        for j = 1:rast_num
            supra_test = true;
            temp_point = [p1(1)+(j/rast_num)*(p2(1)-p1(1)) p1(2)+(j/rast_num)*(p2(2)-p1(2))];
            temp_good1 = pointobscheck(obs_list,temp_point);
            supra_test = supra_test && temp_good1;
            if(~supra_test), break; end %leave loop early if bad
        end
    end
end