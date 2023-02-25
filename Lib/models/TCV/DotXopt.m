%%
function xdot = DotXopt(x,yc,params)

    % init
    xdot = zeros(params.dim_state_op,1);
    Gamma = params.Gamma;
    Anstar = params.Anstar;  
    range = params.dim_state_r+params.dim_state_c+1:params.dim_state_r+params.dim_state_c+params.dim_state_op;
    xop = x(range);
    u = yc + Anstar*xop;

    for i=1:params.m
        if abs(u(i))>params.InputBound
            Grad = params.R(i)*(u(i) - params.InputBound)*Anstar(i,:);
            xdot = xdot-Gamma*Grad';
        end
    end

end