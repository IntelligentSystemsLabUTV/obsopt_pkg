%% params_pendulum
% create params structure for pendulum 
function params = params_grizzle

    % pendulum parameters
    params.a = 5;
    params.b = 6.667;
    params.c = 0.4;
    params.d = 0.05;
    params.e = 0.01;
    params.f = 0.05;
    params.g = 0.02;
    params.h = 0.5;
    params.i = 2;
    
    
    params.X(:,1) = [0.2;0.02;0.005];
    
    % position in the state vector of the parameters
    params.estimated_params = [];
end