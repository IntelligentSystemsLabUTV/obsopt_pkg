%%
function J = J_dist(x,Pa,D)

    % init
    N = length(D)/4; 

    % cost 
    J = 1;  
    for i=1:N
        Jtag = 0;
        for j=1:4
%             Jtag = Jtag + (sqrt((x(3*(i-1)+1)-Pa(1,j))^2 + (x(3*(i-1)+2)-Pa(2,j))^2 + (x(3*(i-1)+3)-Pa(3,j))^2) - D(4*(i-1)+j))^2;
            Jtag = Jtag + abs(sqrt((x(3*(i-1)+1)-Pa(1,j))^2 + (x(3*(i-1)+2)-Pa(2,j))^2 + (x(3*(i-1)+3)-Pa(3,j))^2) - D(4*(i-1)+j));
%             Jtag = Jtag + (((x(3*(i-1)+1)-Pa(1,j))^2 + (x(3*(i-1)+2)-Pa(2,j))^2 + (x(3*(i-1)+3)-Pa(3,j))^2) - D(4*(i-1)+j)^2)^2;
        end       
        J = J+Jtag;
    end    

end