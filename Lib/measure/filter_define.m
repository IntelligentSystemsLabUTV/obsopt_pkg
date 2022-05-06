%% function
function [filter, filterScale, reference] = filter_define(Ts,Nts)

    i = 0;
    filterScale(i+1)= 1;
    filter = [];
    
    %% derivative
    if 1
    i = 1;   
    eps1 = 5e-1;
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
    filterScale(i+1)= 1;
    end

    %% integral
    if 0
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
    
    %% reference filter
    eps = 2e0;
    G = tf(eps,[1 eps]);
    SS = ss(G);
    D = c2d(SS,Ts);
    reference.TF = D;
    reference.A = D.A;
    reference.B = D.B;
    reference.C = D.C;
    reference.D = D.D;
    reference.G = G;
    reference.dim = size(D.B,1);
    
end