%%
clc
clear

%% setup model
A = [0  1   0;  ...
     0  0   -1; ...
     0  0   0];     
    
B = [0  -1  0]';
C = [1  0   1];
D = 0;

dim = size(A,1);
    
sysc = ss(A,B,C,D);    
Ts = 2e-1;
Tm = 0.5*Ts;
TM = 2*Ts;
    
sysd = c2d(sysc,Ts);

I = eye(dim);

%% test observability
Oc = obsv(sysc);
Od = obsv(sysd);
  
%% test Schur
if 1
    syms theta [1 5]
    syms s
        
        
    Gamma = [theta(1)   0           0; ...
             0          theta(2)    0; ...
             theta(3)   theta(4)    -theta(5)];       
        
    eA = expm(sysc.A*Ts);
    PHI = (I - Gamma)*eA;    
        
    thetaval = [0.4247    0.8059  -28.7079   -8.2988   -0.0214];  
%     thetaval = [0.8209    0.8622 -202.9812  -10.5832         0];
    PHIval = double(subs(PHI,theta,thetaval));
end

%% RICCATI solve
if 0
    % STEP 1 - INIT
    % solve in beta,TAU
    % init
    setlmis([]); 
    % scalar
    [beta,n,sbeta] = lmivar(1,[1 1]);
    % matrix TAU
    [TAU,n,sTAU] = lmivar(1,[dim 1]); 
    % compose vars to create eq. A.1
    [COND,n,sCOND] = lmivar(3,transpose((A+sbeta*I))*TAU+TAU*(A+sbeta*I));
    
    % constraints: there is no right side (everything zero)    
    % constraints: -TAU < 0
    lmiterm([1 1 1 0],0*I);
    lmiterm([-1 1 1 TAU],I,I,'s');
    % constraints: -beta < 0
    lmiterm([2 1 1 0],0);
    lmiterm([-2 1 1 beta],1,1);    
    % eq. A.1: -COND < 0
    lmiterm([3 1 1 0],0*I);
    lmiterm([-3 1 1 COND],I,I,'s');
          

    % get constraints
    lmis = getlmis;
    % solve
    [alpha,popt]=gevp(lmis,2);

    % test results
    betaout = popt(1);
    TAUout = [  popt(2) popt(3) popt(5); ...
                popt(3) popt(4) popt(6); ...
                popt(5) popt(6) popt(7)];
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
    C = set(P >= I) + set(P <= p*I);
    for i=1:numel(Tcall)
        systmp = c2d(sysc,Tcall(i));
        eA = systmp.A;
        Ep = [Cperp'*eA'*P*eA*Cperp,    zeros(size(Cperp,2),dim); ...
              P*Cperp,                 P];
        C = C + set(Ep >= 2*mu*eye(size(Ep,1)));
    end    
    % define cost function
    O = p;
    % options
    ops = sdpsettings('solver','gurobi','debug',1);

    %%% start iterations
    END = 0;
    while ~END
        
        % solve
        sol = solvesdp(C,O);

        % check feasibility
        if ~FEASIBLE
            END = 1;
            disp('NOT FEASIBLE')
        end

        % define delta
        delta = mu/(sol.p*norm(A)*gamma*exp(betaout*TM));
        TcallD = Tm:2*delta:TM;
        for i=1:numel(TcallD)
            systmp = c2d(sysc,TcallD(i));
            eA = systmp.A;
            Pstar = sol.P;
            Ep = [Cperp'*eA'*Pstar*eA*Cperp,    zeros(size(Cperp,2),dim); ...
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
            [~,kbar] = min(mineigs(:,1));
            taubar = mineigs(kbar,2);

            % add constraint
            systmp = c2d(sysc,taubar);
            eA = systmp.A;
            Pstar = sol.P;
            Ep = [Cperp'*eA'*Pstar*eA*Cperp,    zeros(size(Cperp,2),dim); ...
                  Pstar*Cperp,                  Pstar];
            C = C + set(Ep >= 2*mu*eye(size(Ep,1)));
        end
        
    end

    
end

%% example
if 0
    a1 = [-1 2; 1 -3];
    a2 = [-0.8 1.5; 1.3 -2.7];
    a3 = [-1.4 0.9; 0.7 -2];
    setlmis([]); 
    p = lmivar(1,[2 1]);
    
    lmiterm([1 1 1 0],1) 	% P > I : I 
    lmiterm([-1 1 1 p],1,1) 	% P > I : P 
    lmiterm([2 1 1 p],1,a1,'s') 	% LFC # 1 (lhs) 
    lmiterm([-2 1 1 p],1,1) 	% LFC # 1 (rhs) 
    lmiterm([3 1 1 p],1,a2,'s') 	% LFC # 2 (lhs) 
    lmiterm([-3 1 1 p],1,1) 	% LFC # 2 (rhs) 
    lmiterm([4 1 1 p],1,a3,'s') 	% LFC # 3 (lhs) 
    lmiterm([-4 1 1 p],1,1) 	% LFC # 3 (rhs) 
    lmis = getlmis;
    % solve
    [alpha,popt]=gevp(lmis,3);
end
