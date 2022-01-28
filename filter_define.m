%% function
function filter = filter_define(Ts,Nts)

    filter = [];
    
    %% derivative
    if 1
    i = 1;   
    eps1 = 1e-4;
    G = tf([1e-4 0],[eps1 1]);
    SS = ss(G);
    D = c2d(SS,Ts*Nts);
    filter(i).TF = D;
    filter(i).A = D.A;
    filter(i).B = D.B;
    filter(i).C = D.C;
    filter(i).D = D.D;
    filter(i).G = G;
    end

    %% integral
    if 0
    i = 2;    
    G = tf(1,[1/0.1 1]);
    SS = ss(G);
    D = c2d(SS,Ts*Nts);
    filter(i).TF = D;
    filter(i).A = D.A;
    filter(i).B = D.B;
    filter(i).C = D.C;
    filter(i).D = D.D;
    filter(i).G = G;
    end
end