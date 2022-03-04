%% function
function [filter, filterScale] = filter_define(Ts,Nts)

    i = 0;
    filterScale(i+1)= 1;
    filter = [];
    
    %% derivative
    if 1
    i = 1;   
    eps1 = 1e-2;
    G = tf([1 0],[eps1 1]);
    SS = ss(G);
    D = c2d(SS,Ts);
    filter(i).TF = D;
    filter(i).A = D.A;
    filter(i).B = D.B;
    filter(i).C = D.C;
    filter(i).D = D.D;
    filter(i).G = G;
    filter(i).dim = size(D.B,1);
    filterScale(i+1)= 0;
    end

    %% integral
    if 1
    i = 2;    
    eps2 = 1e2;
    G = tf(1,[eps2 1]);
    SS = ss(G);
    D = c2d(SS,Ts);
    filter(i).TF = D;
    filter(i).A = D.A;
    filter(i).B = D.B;
    filter(i).C = D.C;
    filter(i).D = D.D;
    filter(i).G = G;
    filter(i).dim = size(D.B,1);
    filterScale(i+1)= 1;
    end
    
    %% dim 2
    if 0
    i = 2;   
    eps1 = 1e-4;
    G = tf([eps1 0],[eps1 2*eps1 1]);
    SS = ss(G);
    D = c2d(SS,Ts*Nts);
    filter(i).TF = D;
    filter(i).A = D.A;
    filter(i).B = D.B;
    filter(i).C = D.C;
    filter(i).D = D.D;
    filter(i).G = G;
    filter(i).dim = size(D.B,1);
    filterScale(i+1)= 0;
    end
    
end