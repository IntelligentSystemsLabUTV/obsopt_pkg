%% function 
function params_out  = params_update_runaway(params,x)

    % assign params
    params_out = params;

    params_out.gamma = x(3);
end