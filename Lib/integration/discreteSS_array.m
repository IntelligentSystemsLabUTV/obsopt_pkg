%%
function [y, xk] = discreteSS_array(x0,u, A, B, C, D)

    xk = A*x0 + B*u(:,1,:);    
    y = C*xk+D*u(:,2,:);
    
end