%% fcn
function [c, ceq] = nonlcon_fcn_rover(xopt,xnonopt,obs)    
    
    % create full state vector
    x = zeros(obs.setup.dim_state,1);
    x(obs.setup.opt_vars) = xopt;
    x(obs.setup.nonopt_vars) = xnonopt(1).val;  
    
    % tolerance
    tol = 1e0*obs.init.myoptioptions.ConstraintTolerance;

    % init
    c = [];    

%     tspan_pos = [1, nonzeros(obs.init.Y_space_full_story)'];
%     tspan = obs.setup.time(tspan_pos(1):tspan_pos(end));
%     X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params, obs), tspan , x, obs.setup.odeset);
%     X = X.y(:,obs.init.Y_space_full_story(2:end));

%     normx = vecnorm(X(obs.init.params.pos_p(1),:),2,1);
%     normy = vecnorm(X(obs.init.params.pos_p(2),:),2,1);
%     gamma(:,1) = x(obs.init.params.pos_Gamma(3))+ x(obs.init.params.pos_Gamma(4))*normx;
%     gamma(:,2) = x(obs.init.params.pos_Gamma(3))+ x(obs.init.params.pos_Gamma(4))*normy;    

%     % CUBIC TERMS
    ThetaCubic = x(obs.init.params.pos_Gamma(4:6));
    ThetaCubic_constr_down = -ThetaCubic;
    c = [c; ThetaCubic_constr_down];
    
            
    % cons
    ceq = [];

end