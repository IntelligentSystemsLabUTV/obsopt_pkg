%%
function J = J_dist(x,Pa,D)

    % init
    N = length(D);
    J = 0;

    % 
    for i=1:N
        J = J + abs(sqrt((x(1)-Pa(1,i))^2 + (x(2)-Pa(2,i))^2 + (x(3)-Pa(3,i))^2) - D(i));
    end

end