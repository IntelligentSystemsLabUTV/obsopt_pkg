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
function u = control_TCV(t,drive,params,obs)

    % check if the input is enabled
    if params.input_enable
        
        % !REMARK!
        % simply uncomment or add the law you need, according to params

%         for i=1:params.m
%             tmpstr =  ['u(', num2str(i) ',:) = ', num2str(i), '*sin(', num2str(i), '*t);'];
%             eval(tmpstr); 
%         end            
        u(:,1) = ones(1,length(t));
    else
        u = zeros(params.dim_input,1);
    end
end