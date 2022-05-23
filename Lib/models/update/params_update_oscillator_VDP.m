%% function 
function params = params_update_oscillator_VDP(params,x)
    % update the nonlinearity
    params.mu = x(3);
end