%%
function y = discreteSS(x0,u, A, B, C, D)

    dim_filter = size(B,1);
    dim_input = size(B,2);

    xk = x0;
    for i=1:dim_filter
        xk = A*xk + B*u(:,i);
    end
    y = C*xk+D*u(end);
    
end