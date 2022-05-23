%% measure function
function y = measure_battery(x,params)
    y = params.Uoc - x(2) - params.R1*params.u;
end