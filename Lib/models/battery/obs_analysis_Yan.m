%% observability analysis - Yan model
function out = obs_analysis_Yan(varargin)

    %% init stuff 
    % autonomous or not
    aut = varargin{1};        
    
    % obs as input if present    
    obs = varargin{2};
    
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
    x10 = 0.9;
    x20 = 0.01;        

    % testing - init with correct vals
    params.input_current = input_data.Current;
    params.Voc = spline(params.input_soc, params.input_OCV, x10);
    params.R0 = spline(params.input_soc, params.input_R0, x10);
    params.R1 = spline(params.input_soc, params.input_R1, x10);
    params.C1 = spline(params.input_soc, params.input_C1, x10);

    % other params
    params.C_n = 28 * 3600; 
    params.eta = 1;
    params.Ts = 1;

    % generate state
    syms x [1 5]
    syms u

    digits(2)
    dim = length(x);
    eqs = [1 2 5];
    idx = [1:2,8:10];
    tol = 1e-5;
%     Niter = obs.setup.Niter-1;
    Niter = 100;

    %% model equations - f
    % Zk (SOC)
    out.f(1,1) = vpa(x(1));
    % V1 (voltage RC)
    R1 = x(3) + obs.init.params.beta_R1*x(1);
    C1 = x(4) + obs.init.params.beta_C1*x(1);
    out.f(2,1) = vpa(exp(-params.Ts/(R1*C1))*x(2));

    % params
    out.f(3,1) = vpa(x(3));
    out.f(4,1) = vpa(x(4));
    out.f(5,1) = vpa(x(5));
    
    %% model equations - g
    % Zk (SOC)
    out.g(1,1) = vpa(-u*params.Ts/params.C_n);
    % V1 (voltage RC)
    out.g(2,1) = vpa(R1*(1 - exp(-params.Ts/(R1*C1)))*u);

    % params
    out.g(3,1) = 0;
    out.g(4,1) = 0;
    out.g(5,1) = 0;    
    
    %% output mapping
    Voc = obs.init.params.alpha_Voc+obs.init.params.beta_Voc*x(1);
    R0 = x(5) + obs.init.params.beta_R0*x(1);    
    V = x(2);
    I = u;
    out.h(1,1) = (Voc - V - I*R0);
    
    if aut
        %% ORC - autonomous
        % subs u=0
        out.h_aut = subs(out.h,u,0);

        out.Theta_aut = [   out.h_aut; ...
                            Lie_bracket(out.h_aut,out.f,x); ...
                            Lie_bracket(Lie_bracket(out.h_aut,out.f,x),out.f,x); ...
                            Lie_bracket(Lie_bracket(Lie_bracket(out.h_aut,out.f,x),out.f,x),out.f,x); ...
                            Lie_bracket(Lie_bracket(Lie_bracket(Lie_bracket(out.h_aut,out.f,x),out.f,x),out.f,x),out.f,x); ...
                            Lie_bracket(Lie_bracket(Lie_bracket(Lie_bracket(Lie_bracket(out.h_aut,out.f,x),out.f,x),out.f,x),out.f,x),out.f,x); ...
                        ];

        for i=1:length(out.Theta_aut)
            out.dTheta_aut(i,:) = gradient_sym(out.Theta_aut(i),x);
        end

        %% check ORC on obs - aut
        try
            for i=1:Niter
                state = reshape(obs.init.X(1).val(idx,i),1,dim);
                tmp = double(vpa(subs(out.dTheta_aut(eqs),x,state),2));
                out.rank_aut(i,1) = rank(tmp,tol);
                out.eig_aut(i,:) = eig(tmp*tmp');
            end
        catch
            disp('NO obs provided') 
        end
    else
        %% ORC - input        
        out.Theta = [   out.h; ...
                        Lie_bracket(out.h,out.f,x); ...
                        Lie_bracket(out.h,out.g,x); ...
                        Lie_bracket(Lie_bracket(out.h,out.f,x),out.g,x); ...
                        Lie_bracket(Lie_bracket(out.h,out.g,x),out.f,x); ...                        
                        Lie_bracket(Lie_bracket(out.h,out.f,x),out.f,x); ...
                        Lie_bracket(Lie_bracket(Lie_bracket(out.h,out.f,x),out.f,x),out.f,x); ...
                        Lie_bracket(Lie_bracket(Lie_bracket(Lie_bracket(out.h,out.f,x),out.f,x),out.f,x),out.f,x);
                    ];

        for i=1:length(out.Theta)
            out.dTheta(i,:) = gradient_sym(out.Theta(i),x);
        end

        %% check ORC on obs - input
        try
            for i=1:Niter
                state = reshape(obs.init.X(1).val(idx,i),1,dim);
                input = obs.init.input_story(1).val(i);
                tmp = vpa(subs(out.dTheta(:,eqs),x,state),2);
                tmp = double(vpa(subs(tmp,u,input),2));
                out.rank(i,1) = rank(tmp,tol);
                out.eig(i,:) = eig(tmp*tmp');
            end
        catch
            disp('NO obs provided') 
        end
    end
end