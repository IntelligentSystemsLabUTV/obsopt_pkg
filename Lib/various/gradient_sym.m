%% gradient of sym function
function grad = gradient_sym(f,x)

    n = length(x);
    Nf = length(f);
    
    for i=1:Nf
        for j=1:n
            grad(i,j) = diff(f(i),x(j));
        end
    end

end