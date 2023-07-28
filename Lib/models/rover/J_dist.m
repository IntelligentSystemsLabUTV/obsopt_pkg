%%
function J = J_dist(x,Pa,D)

    % init
    N = length(D)/4;
    J = 0;

    % 
    for i=1:N
        for j=1:4
            J = J + abs(sqrt((x(3*(i-1)+1)-Pa(1,j))^2 + (x(3*(i-1)+2)-Pa(2,j))^2 + (x(3*(i-1)+3)-Pa(3,j))^2) - D(4*(i-1)+j));
        end       
    end

    % for i=1:N
    %     J = J + abs(sqrt((x(1)-Pa(1,i))^2 + (x(2)-Pa(2,i))^2) - D(i));
    % end

end