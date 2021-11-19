%% params_pendulum
% create params structure for pendulum 
function params = params_double_pendulum_mass

    % pendulum parameters
    params.Lt1 = 1;
    params.M1 = 1;
    params.Lt2 = 2;
    params.M2 = 0.5;
    params.g = 9.81;
    params.c1 = 0.1;
    params.c2 = 0.1;
    
    
    params.X(:,1) = [-pi/2+0.3;-pi/2-0.2;0.1;-0.2; params.M1; params.M2];
    
    % position in the state vector of the parameters
    params.estimated_params = [3, 4];
end