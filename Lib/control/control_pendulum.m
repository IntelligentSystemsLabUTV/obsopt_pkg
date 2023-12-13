function u = control(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable

        u(1,:) = 2*sin(5*t);
        u(2,:) = 4*cos(10*t);
        
    end

end