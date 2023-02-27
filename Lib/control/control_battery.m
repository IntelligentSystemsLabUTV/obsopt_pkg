%% CONTROL
% file: control.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function implements the control law
% INPUT:
% t: time instant
% drive: error variable
% params: structure with all the necessary parameters 
% OUTPUT:
% u: control variable
function u = control_battery(t,drive,params,obs)

    % init input
    u = zeros(1,length(t));
    % params.dim_input

    % compute the time index
    for i=1:length(t)
        tdiff = obs.setup.time-t(i);   
        pos(i) = find(abs(tdiff) == min(abs(tdiff)),1,'first');    
        pos(i) = max(1,pos(i));        
    end 

    % check if the input is enabled
    if params.input_enable
        
        % from input data        
        u = params.u_sim(:,pos);
        
           
    end
end