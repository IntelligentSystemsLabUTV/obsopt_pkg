%% function
function Lf_g = Lie_bracket(f,g,x)

    % reshape g
    g = reshape(g,length(g),1);
    
    % lie brackert
    Lf_g = gradient_sym(f,x)*g;

end