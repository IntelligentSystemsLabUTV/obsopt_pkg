function u = control(t,drive,params,obs)

    % init input
    u = zeros(params.dim_input,length(t));

    if params.input_enable
        u(1) = -2*params.rhox*params.wnx*drive(3) - params.wnx^2*drive(1);
        u(2) = -2*params.rhoy*params.wny*drive(4) - params.wny^2*drive(2);
    end


end