%%
function dc = dcgain_ss(A,B,C,D)

    dc = -(C(1)*(A(4)*B(1)-A(3)*B(2)) + C(2)*(A(1)*B(2)-A(2)*B(1)))/(A(1)*A(4)-A(2)*A(3));
    
end