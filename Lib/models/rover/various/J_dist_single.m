%%
function J = J_dist_single(x,Pa,D)


    % cost 
    J = 1;  
    Jtag = 0;
    for j=1:4
        Jtag = Jtag + (((x(1)-Pa(1,j))^2 + (x(2)-Pa(2,j))^2 + (x(3)-Pa(3,j))^2) - D(j)^2)^2;
    end       
    J = J+Jtag;   

end