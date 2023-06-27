%%
clc
clear

%% system
for i=1:100
    A = [-0.02  -1.4    9.8;    ...
         -0.01  -0.4    0;      ...
         0      1       0];
    B = [9.8 6.3 0]';
    C = [1 0 0];
    Ts = 1 + 2*rand();
    sysc = ss(A,B,C,0);
    
    %% perp and P
    eA = expm(A*Ts);
    eA_ = expm(-A*Ts);
    Cp = null(C);
    
    % his test
    P = [0.8334 -0.0041 0.8333;     ...
        -0.0041 2.0116  -0.0359;    ...
        0.8333  -0.0359 0.8341];
    
    % my test
    P = [   3.4080e+00  -5.7473e-01  -1.4416e-01; ...
            -5.7473e-01   7.2290e+00  -1.9968e+00; ...
            -1.4416e-01  -1.9968e+00   4.9967e+00];
    
    
    %% compute gain
    T1 = Cp*pinv(Cp'*eA_'*P*eA_*Cp)*Cp';
    T2 = (eA_'*P*eA_)'*C';
    K = (C'-T1*T2)*pinv(C*C');
    
    %% dynamics matrix
    PHI = (eye(3)-K*C)*eA;    
    eig_store(:,i) = real(eig(PHI));
    Schur_store(i) = prod(any(abs(eig_store(:,i))>1));
end