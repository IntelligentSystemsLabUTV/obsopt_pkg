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
function u = control(t,drive,params,obs)

    % init input
    u = zeros(1,length(t));
    % params.dim_input
        
    % type of input
    type = 4;
    
    % params
    period = 100;
    DC = 0.1;

    % check if the input is enabled
    if params.input_enable
        
        switch type                

            %%%% control law - battery %%%%
            % HCCP
            case 1 
                u(1,:) = -1 * interp1(params.input_time,params.input_current,t,'previous');
            
            %%%% control law - battery %%%%
            % sum of sins
            case 2
                u(1,:) = 5*sin(0.1*t) + 1*sin(0.5*t) + 0.2*sin(1*t) + (1*ceil(t/period));
            
            case 3           
                if mod(t,period) > period*DC
                   u(1,:) = 1; 
                else
                   u(1,:) = 0;
                end
                
            case 4    
                len = min(length(params.input_current_modular_time_slown_dense),length(params.input_current_modular_slown));
                u(1,:) = -params.InputAmplitude*interp1(params.input_current_modular_time_slown_dense(1:len),params.input_current_modular_slown(1:len),mod(t,params.input_current_modular_period_slown),'previous');
                
                if 0
                    tspan_start = [1000:2000:10000];
                    tspan_stop = tspan_start + 200;                    
                    mask_start = find(t(end)>tspan_start);
                    mask_stop = find(t(end)<tspan_stop);
                    flag = ~isempty(intersect(mask_start,mask_stop));
                    if flag
                        u(1,:) = 0;
                    end                
                end
            
            otherwise
                disp('no input law selected')
        end
           
    end
end