function u = control(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable
        u(1) = -2*params.rhox*params.wnx*drive(2) - 1*params.wnx^2*drive(1);
        u(2) = -2*params.rhoy*params.wny*drive(7) - 1*params.wny^2*drive(6);
    end


end