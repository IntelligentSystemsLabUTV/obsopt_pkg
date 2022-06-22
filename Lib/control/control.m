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
function u = control(t,drive,params)

    % init input
    u = zeros(params.dim_input,length(t));

    % check if the input is enabled
    if params.input_enable
        
        % !REMARK!
        % simply uncomment or add the law you need, according to params

        %%%% control law - battery %%%%
        % sine wave
        u(1,:) = sin(t/params.period);
        % PWM
%         period = params.period;
%         mod_t = mod(t,period);
%         pos_less = find(mod_t<period/2);
%         pos_geq = find(mod_t>=period/2);                          
%         u(1,pos_less) = 0; 
%         u(1,pos_geq) = 1;
           
    end
end