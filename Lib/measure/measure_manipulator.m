%% measure function
function [y, yall] = measure_manipulator(x,params)
    theta = x(params.observed_state);
    
    % position origin
    yall(1,1) = 0;
    yall(2,1) = 0;
    
    % position first link
    yall(1,2) = params.Lt1*cos(theta(1));
    yall(2,2) = params.Lt1*sin(theta(1));
    
    % position end effector
    yall(1,3) = params.Lt1*cos(theta(1))+params.Lt2*cos(theta(2));
    yall(2,3) = params.Lt1*sin(theta(1))+params.Lt2*sin(theta(2));
    
    % measured - end effector
%     y(1,1) = yall(1,3);
%     y(2,1) = yall(2,3);
    
    % measured - joint space
    y(:,1) = theta;
end