%% cost function with newton 
function [J, GJ, HJ] = J_fcn(x,y,P_a,D)

    % iterate    
    tmp = (sqrt( (P_a(1,:)-x).^2 + (P_a(2,:)-y).^2 ) - D);    
    J = sum(tmp);
end