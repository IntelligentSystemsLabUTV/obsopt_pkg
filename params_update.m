%% function 
function params = params_update(params,x)
    params.K1 = x(5);
    params.K2 = x(6);
    params.K3 = x(7);
    params.K4 = x(8);
    params.K5 = x(9);
    params.K6 = x(10);
end