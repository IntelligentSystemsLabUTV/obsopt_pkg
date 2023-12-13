function out = obs_analysis_unicycle(varargin)

    %% init stuff      
    
    % obs as input if present    
    obs = varargin{1};        

    % generate state
    syms x [1 3]
    syms u [1 2]

    digits(2)
    dim = length(x);
    eqs = [1 2 3];    
    tol = 1e-5;        

    %% model equations - f    
    out.f(1,1) = 0;        
    out.f(2,1) = 0;
    out.f(3,1) = 0;    
    
    %% model equations - g u1
    out.g(1,1) = cos(x(3));    
    out.g(2,1) = sin(x(3));    
    out.g(3,1) = 0;
    
    %% model equations - g u2
    out.g(1,2) = 0;    
    out.g(2,2) = 0;    
    out.g(3,2) = 1;

    %% output mapping        
    out.h(1,1) = x(1);
    out.h(2,1) = x(2);
    
    
    %% ORC - input        
    out.Theta = [   out.h(1); ...
                    out.h(2); ...
                    Lie_bracket(out.h(1),out.g(eqs,1),x(eqs)); ...
                    Lie_bracket(out.h(2),out.g(eqs,1),x(eqs))
                ];

    for i=1:length(out.Theta)
        out.dTheta(i,:) = gradient_sym(out.Theta(i),x(eqs));
    end    
end