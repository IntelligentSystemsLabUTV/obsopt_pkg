%%
function J = Jcost(x,yc,params) 

    % init
    J = 0;    
    Anstar = params.Anstar;  
    range = params.dim_state_r+params.dim_state_c+1:params.dim_state_r+params.dim_state_c+params.dim_state_op;
    xop = x(range);
    u = yc + Anstar*xop;

    for i=1:params.m
        if abs(u(i))>params.InputBound            
            J =  J + 0.5*params.R(i)*u(i)^2;            
        end
    end

end