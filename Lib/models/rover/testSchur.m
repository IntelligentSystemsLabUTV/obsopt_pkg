%%
clc
clear

%% setup model
% sferlazza
if 0
    A = [-0.02  -1.4    9.8;    ...
         -0.01  -0.4    0;      ...
         0      1       0];
    B = [9.8 6.3 0]';
    C = [1 0 1];
    D = 0;
end

% Localization
if 0
    af = 1;
    A = [0  1   0   0; ...
         0  0   -1  1; ...
         0  0   0   0; ...
         0  0   0   -af];
    B = [0 0 0 af]';
    C = [1  0   0   0; ...
         0  1   0   0; ...
         0  0   0   1];
    D = 0;
end

% error dynamics - with PD
if 1
    % case same observer as plant
    A = [0  1   0 ; ...
         0  0   -1; ...
         0  0   0];
    B = [0 0 0]';
    C = [1  0   0; ...
         0  1   0];
    D = 0;   
end

% error dynamics - without PD
if 0
    % case same observer as plant
    A = [0  1   0 ; ...
         0  0   -1; ...
         0  0   0];
    B = [0 0 0]';
    C = [1  0   0];
    D = 0;   
end

% plant dynamics
if 0
    % case same observer as plant
    A = [0  1   0 ; ...
         0  0   0; ...
         0  0   0];
    B = [0 1 0]';
    C = [1  0   0; ...
         0  0   0];
    D = [0; 0];   
end

dim = size(A,1);
    
sysc = ss(A,B,C,D);    

% sampling
Tm = 0.1;
TM = 0.3;
I = eye(dim);

%% test observability
Oc = obsv(sysc);
  
%% test Schur
if 1

    Ts = 2e-1;
    sysd = c2d(sysc,Ts);
    Od = obsv(sysd);
    syms theta [1 5]
    syms s
        
        
    Gamma = [1-theta(1)   0           0; ...
             0          1-theta(2)    0; ...
             -theta(3)   -theta(4)    1];       
        
    eA = expm(sysc.A*Ts);
%     eA = eye(3)+sysc.A*Ts;
    PHI = Gamma*eA;    
        
    thetaval = [1.9971         0    0.0875         0         0];  % true x1x2
%     thetaval = [0.9927    0.1010    1.3028   -0.0441         0];  % noise
    PHIval = double(subs(PHI,theta,thetaval));
end

%% Sferlazza algorithm
if 0

    % obsv analysis    
    deltatau = 1e-3:1e-2:10e0;
    for i=1:numel(deltatau)
        sysd = c2d(sysc,deltatau(i));
        Od = obsv(sysd);
        S = svd(Od);
        min_obsv_svd(i) = max(S);
    end

    % STEP 1 - INIT
    % solve in beta,TAU
    % init
    setlmis([]); 
    % scalar
    [beta,n,sbeta] = lmivar(1,[1 1]);
    % matrix TAU
    [TAU,n,sTAU] = lmivar(1,[dim 1]);         
    
    % constraints: there is no right side (everything zero)    
    % constraints: -TAU < 0
    lmiterm([1 1 1 0],0*I);
    lmiterm([-1 1 1 TAU],I,I,'s');
    % constraints: -beta < 0
    lmiterm([2 1 1 0],0);
    lmiterm([-2 1 1 beta],1,1);        
    % eq. A.1: -COND < 0
    lmiterm([3 1 1 0],0*I);
    lmiterm([-3 1 1 TAU],transpose((A+beta*I)),I,'s');
    lmiterm([-3 1 1 TAU],I,(A+beta*I),'s');
          

    % get constraints
    lmis = getlmis;
    % get number of vars
    ndec = decnbr(lmis);

    % solve
%     [alpha,popt]=gevp(lmis,2);
    [alpha,popt]=feasp(lmis);    

    % test results
    betaout = dec2mat(lmis,popt,beta);
    TAUout = dec2mat(lmis,popt,TAU);    
    CONDout = transpose((A+betaout*I))*TAUout+TAUout*(A+betaout*I);

    try chol(CONDout);
        disp('Matrix is symmetric positive definite.')
    catch ME
        disp('Matrix is not symmetric positive definite')
    end

    % eigs
    lambdam = min(eig(TAUout));
    lambdaM = max(eig(TAUout));
    gamma = sqrt(lambdaM/lambdam);    

    %%% STEP 2
    % compute constant vals
    mu = 1e0;
    Cperp = null(C);
    % init set for tau
    Tcall = [Tm TM];
    % define vars
    p = sdpvar(1);
    P = sdpvar(dim); 

    % define constraints - initial
    Con = [];
    Con = [Con; P >= 1e0*I];
    Con = [Con; P <= p*I];
    for i=1:numel(Tcall)        
        eA = expm(A*Tcall(i));
        eA_ = expm(-A*Tcall(i));
        Ep = [Cperp'*eA_'*P*eA_*Cperp,    Cperp'*P'; ...
              P*Cperp,                 P];
        Con = [Con; Ep >= 2*mu*eye(size(Ep,1))];
    end    
    % define cost function
    O = p;
    % options
    ops = sdpsettings('solver','mosek','debug',1);

    %%% start iterations
    END = 0;
    while ~END
        
        % solve
        sol = solvesdp(Con,O,ops);

        % check feasibility
        if sol.problem
            END = 1;
            disp('NOT FEASIBLE')
        else
            % values
            Pstar = value(P);
            pstar = value(p);

             % define delta
            delta = max(1e-4,mu/(pstar*norm(A)*gamma*exp(betaout*TM)));         
            TcallD = Tm:2*delta:TM;
            for i=1:numel(TcallD)            
                % eq. 13 
                eA = expm(A*TcallD(i));         
                eA_ = expm(-A*TcallD(i));         
                Ep = [Cperp'*eA_'*Pstar*eA_*Cperp,    Cperp'*Pstar'; ...
                      Pstar*Cperp,                  Pstar];
                mineigs(i,1) = min(eig(Ep));
                mineigs(i,2) = TcallD(i);                        
            end
    
            % check if algorithm ok - all eigs are above mu
            if prod(mineigs(:,1) > mu)
                END = 1;
                disp('SOLUTION FOUND (PSTAR)')
            else
                % find worst case
                kbar = find(mineigs(:,1) == min(mineigs(:,1)),1,'first');
                taubar = mineigs(kbar,2);
    
                % add constraint            
                eA = expm(A*taubar);                   
                eA_ = expm(-A*taubar);
                Ep = [Cperp'*eA_'*P*eA_*Cperp,    Cperp'*P; ...
                      P*Cperp,                  P];
                Con = [Con;Ep >= 2*mu*eye(size(Ep,1))];
            end
        end 
    end  

    %% test result
    if ~sol.problem
        deltacheck = Tm:1e-3:TM;
        for i=1:numel(deltacheck)        
            Ts = deltacheck(i);        
            
            %% perp and P
            eA = expm(A*Ts);
            eA_ = expm(-A*Ts);                                   
            
            %% compute gain
            T1 = Cperp*pinv(Cperp'*eA_'*Pstar*eA_*Cperp)*Cperp';
            T2 = (eA_'*Pstar*eA_)'*C';
            K = (C'-T1*T2)*pinv(C*C');
            
            %% dynamics matrix
            PHI = (eye(dim)-K*C)*eA;    
            eig_store(:,i) = real(eig(PHI));
            Schur_store(i) = prod(any(abs(eig_store(:,i))>1));
    end
end
end


