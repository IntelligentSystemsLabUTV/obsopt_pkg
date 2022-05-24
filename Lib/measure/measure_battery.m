%% measure function
function y = measure_battery(x,params,t)

    % control
    params.u = params.input(t,x,params);
    
    % measure
    y = params.Uoc - x(2) - params.R1*params.u;
end