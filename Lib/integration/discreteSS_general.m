%%
function [y, xk] = discreteSS_general(k, x0, u, A, B, C, D)

    xk = A*x0 + B*u(:,k);    
    y = C*xk+D*u(:,k+1);
    
end