%% function 
function params = params_update_doublependulum_control(params,x)
    params.K1 = x(5);
    params.K2 = x(6);
    params.K3 = x(7);
    params.K4 = x(8);
    params.K5 = x(9);
    params.K6 = x(10);
    params.K7 = x(11);
    params.K8 = x(12);

    params.K = [params.K1,params.K2,params.K3,params.K4;...
                params.K5,params.K6,params.K7,params.K8];

%       params.K = [params.K1,params.K2;...
%                 params.K3,params.K4];
end