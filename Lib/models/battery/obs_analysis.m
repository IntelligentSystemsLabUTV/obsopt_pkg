function out = obs_analysis(varargin)

    %% init stuff      
    
    % obs as input if present    
    obs = varargin{1};
    
    % load data
    input_data = load('data/ECM_parameters.mat');
    
    % Loading input signals and parameter data    
    params.input_time = input_data.Time;    
    params.input_OCV = input_data.OCV;
    params.input_soc = input_data.SOC;
    params.input_R0 = input_data.R0;
    params.input_R1 = input_data.R1;
    params.input_C1 = input_data.C1;

     % initial SOC
    x10 = obs.init.X(1).val(1,1);
    x20 = obs.init.X(1).val(2,1);        

    % testing - init with correct vals
    params.input_current = input_data.Current;
    params.Voc = obs.init.X(1).val(3,1);
    params.R0 = obs.init.X(1).val(4,1);
    params.R1 = obs.init.X(1).val(5,1);
    params.C1 = obs.init.X(1).val(6,1);
    
    % coefficients
    alpha = 1*[obs.setup.params.alpha_Voc, obs.setup.params.alpha_R0, obs.setup.params.alpha_R1, obs.setup.params.alpha_C1];
    beta = 1*[obs.setup.params.beta_Voc, obs.setup.params.beta_R0, obs.setup.params.beta_R1, obs.setup.params.beta_C1];

    % other params
    params.C_n = 28 * 3600; 
    params.eta = 1;
    params.Ts = 1;

    % generate state
    syms x [1 10]
    syms u
    
    % assumption
    assume(x,'real');
    assume(u,'real');

    digits(2)
    dim = length(x);
    eqs = [1 2 6];
    idx = [1:2,7:10,11:14];
    tol = 1e-3;
%     Niter = obs.setup.Niter-1;
    Niter = 50;
    
    % params models
    Voc = x(3) + x(7)*x(1);
    R0 = x(4) + x(8)*x(1);    
    R1 = x(5) + x(9)*x(1);
    C1 = x(6) + x(10)*x(1);

    %% model equations - f
    % Zk (SOC)
    out.f(1,1) = vpa(x(1));
    % V1 (voltage RC)    
    out.f(2,1) = vpa(exp(-params.Ts/(R1*C1))*x(2));

    % params    
    out.f(3,1) = vpa(x(3));
    out.f(4,1) = vpa(x(4));
    out.f(5,1) = vpa(x(5));
    out.f(6,1) = vpa(x(6));
    out.f(7,1) = vpa(x(7));
    out.f(8,1) = vpa(x(8));
    out.f(9,1) = vpa(x(9));
    out.f(10,1) = vpa(x(10));
    
    %% model equations - g
    % Zk (SOC)
    out.g(1,1) = vpa(-params.Ts/params.C_n);
    % V1 (voltage RC)
    out.g(2,1) = vpa(R1*(1 - exp(-params.Ts/(R1*C1))));

    % params
    out.g(3,1) = 0;
    out.g(4,1) = 0;
    out.g(5,1) = 0;
    out.g(6,1) = 0;
    out.g(7,1) = 0;
    out.g(8,1) = 0;
    out.g(9,1) = 0;
    out.g(10,1) = 0;
    
    %% output mapping    
    V = x(2);
    I = u;
    y = (Voc - V - I*R0);
    out.h(1,1) = y;
    
    
    %% ORC - input        
    out.Theta = [   out.h; ...
                    Lie_bracket(out.h,out.f(eqs),x(eqs)); ...
                    Lie_bracket(out.h,out.g(eqs),x(eqs)); ...
                    Lie_bracket(Lie_bracket(out.h,out.f(eqs),x(eqs)),out.g(eqs),x(eqs)); ...
                    Lie_bracket(Lie_bracket(out.h,out.g(eqs),x(eqs)),out.f(eqs),x(eqs)); ...                        
                    Lie_bracket(Lie_bracket(out.h,out.f(eqs),x(eqs)),out.f(eqs),x(eqs)); ...
                    Lie_bracket(Lie_bracket(Lie_bracket(out.h,out.f(eqs),x(eqs)),out.f(eqs),x(eqs)),out.g(eqs),x(eqs)); ...                    
                    Lie_bracket(Lie_bracket(Lie_bracket(out.h,out.f(eqs),x(eqs)),out.g(eqs),x(eqs)),out.f(eqs),x(eqs)); ...
                    Lie_bracket(Lie_bracket(Lie_bracket(out.h,out.g(eqs),x(eqs)),out.f(eqs),x(eqs)),out.f(eqs),x(eqs))
                ];

    for i=1:length(out.Theta)
        out.dTheta(i,:) = gradient_sym(out.Theta(i),x(eqs));
    end

    %% check ORC on obs - input
    try
        for i=1:Niter
            % define state
            state = reshape(obs.init.X(1).val(idx,i),1,dim);
            
            % if you want to use constant params
            state(3:6) = alpha;
            state(7:10) = beta;
                        
            % get input
            input = obs.init.input_story(1).val(i);
            
            % compute O
            tmp = vpa(subs(out.dTheta,x,state),2);
            tmp = double(vpa(subs(tmp,u,input),2));
            
            % get ORC
            out.rank(i,1) = rank(tmp,tol);
            out.eig(i,:) = svd(tmp);
            e = out.eig(i,:);
            out.cond(i) = max(e(e>tol))/min(e(e>tol));
        end
    catch
        disp('NO obs provided') 
    end
end