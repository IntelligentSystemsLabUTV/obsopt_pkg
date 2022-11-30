%% function
function grad = gradient_sym(f,x)

    % state dim
    dim = length(x);
    
    % gradient
    for i=1:dim
       grad(1,i) = diff(f,x(i));
    end

end