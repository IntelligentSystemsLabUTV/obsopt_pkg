%% measure function
function u = control(x,params)

    % control law
    u = params.K*x(2);
    
    % input enable
    u = params.input_enable*u;
end