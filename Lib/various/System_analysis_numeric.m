%% script for the theoretical test

% stuff
close all
clear 
clc

% flags
filter = 1;
numeric = 0;
symbolic = 1;
offset = 10;
stab = -1;

% init observer buffer
Nw = 4;
Nts = 3;

% set sampling time
Ts = 5e-2;

% set initial and final time instant
t0 = 0;
tend = (Nts*Nw+offset+1)*Ts;

params_init = @params_mockup;
model = @model_mockup;
measure = @measure_general;
ode = @oderk4;

% define observer
params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0,'noise_spec',[0, 0],...
        'model',model,'measure',measure,'StateDim',2,'ObservedState',[1],'ode',ode,...
        'input_enable',0,'dim_input',1,'input_law',[],'params_init',params_init);
    

% create observer class instance. For more information on the setup
% options check directly the class constructor
obs = obsopt_general_adaptive_flush('Nw',Nw,'Nts',Nts,'ode',ode, 'optimise', 0, ...    
      'params',params, 'filters',[1,filter,0,0],'Jdot_thresh',0.9,'MaxIter',200,...
      'AlwaysOpt',0,'print',0,'SafetyDensity',5,'AdaptiveHist',[5e-3, 1e-2],...
      'AdaptiveSampling',1, 'FlushBuffer', 1, 'Jterm_store',0, 'opt', @fminsearch);


% define evaluation grid
Tgran = [1e-2;1e-2];
x_range = [0.6*params.X, 1.4*params.X];
for i=1:params.StateDim
    x_grid(i).val = x_range(i,1):Tgran(i):x_range(i,2);
end
[XVAL,THETA] = meshgrid(x_grid(1).val,x_grid(2).val);


if numeric
    %% NUMERIC ANALYSIS %%
    % compute cost function
    V_num = zeros(length(x_grid(1).val),length(x_grid(2).val));
    for a=1:length(x_grid(1).val)
        for b=1:length(x_grid(2).val)

            obs.init.X_est(:,1) = [x_grid(1).val(a); x_grid(2).val(b)];

            % simulate the system 
            for i = 1:obs.setup.Niter

                % set current interation in the class
                obs.init.ActualTimeIndex = i;
                obs.init.t = obs.setup.time(i);

                %%%%%%%%%%%%%%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%%
                % forward propagation of the previous estimate
                if(obs.init.ActualTimeIndex > 1)
                    % input
                    obs.init.params.u = obs.setup.params.input(:,i-1);

                    % true system
                    X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X(:,obs.init.ActualTimeIndex-1));   
                    obs.init.X(:,obs.init.ActualTimeIndex) = X.y(:,end);

                    % real system
                    X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.init.params), obs.setup.tspan, obs.init.X_est(:,obs.init.ActualTimeIndex-1));
                    obs.init.X_est(:,obs.init.ActualTimeIndex) = X.y(:,end);
                end

                %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%   
                % here the noise is added
                y_meas = obs.setup.measure(obs.init.X(:,obs.init.ActualTimeIndex),obs.init.params);

                %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obs = obs.observer(obs.init.X_est(:,obs.init.ActualTimeIndex),y_meas);

            end

            % stop for zero val
            if obs.init.X_est(:,1) == obs.init.X(:,1)
               check = 0; 
            end

            % get the cost function
            obs.init.temp_x0 = obs.init.X_est(:,1);
            obs.init.Y_space = (Nts+offset):Nts:(Nts*(Nw)+offset);
    %         obs.init.Yhat_full_story = obs.init.Y_full_story;
            obs.init.Y = obs.init.Y_full_story(:,:,obs.init.Y_space);
            V_num(a,b) = obs.cost_function(obs.init.temp_x0);
        end
    end
    
    % match meshgrid
    V_num = V_num';

    % gradient analysis
    [GX, GTHETA] = gradient(V_num,Tgran(1),Tgran(2));

    % hessian analysis
    [GXX, GXTHETA] = gradient(GX,Tgran(1),Tgran(2));
    [GTHETAX, GTHETATHETA] = gradient(GTHETA,Tgran(1),Tgran(2));
    V_hess_val_num = {GXX,GXTHETA;GTHETAX,GTHETATHETA};

    H_map = zeros(size(V_num));
    H_map_bool = zeros(size(V_num));
    for i=1:length(x_grid(1).val)
        for j=1:length(x_grid(2).val)
            H = [GXX(j,i), GXTHETA(j,i); GTHETAX(j,i), GTHETATHETA(j,i)];
            tmp_eig = eig(H);
            IsPositiveDefinite = ~(any(tmp_eig<0));
            if IsPositiveDefinite
                H_map(j,i) = norm(tmp_eig);
                H_map_bool(j,i) = 1;
            else
                H_map(j,i) = -norm(tmp_eig);
                H_map_bool(j,i) = NaN;
            end
        end
    end
end
if symbolic
    %% SYMBOLIC ANALYSIS
    % sym vars
    syms y [1 Nw] real
    syms y_dot [1 Nw] real
    syms x0 real
    syms theta real
    syms T real
    syms V real
    syms k real

    % model
    theta_true = obs.init.X(2,1);
    x0_true = obs.init.X(1,1);
    T_true = Ts;
    state = [x0, theta];

    % flow
    phi = x0.*exp(stab*theta*k*T);
    phi_dot = stab*x0.*theta*exp(stab*theta*k*T);
    phi_fun = symfun(phi,[x0, theta, k, T]);
    phi_fun_dot = symfun(phi_dot,[x0, theta, k, T]);

    % check pivots around y
    y_star = zeros(1,Nw);
    for i=1:Nw
        y_star(i) = double(phi_fun(x0_true, theta_true, i*Nts+offset-1,T_true));
        y_star_dot(i) = double(phi_fun_dot(x0_true, theta_true, i*Nts+offset-1,T_true));
    end

    % define function
    V = sym(0);
    for i=1:Nw
       eval(['tmp = (y',num2str(i),' - phi_fun(x0,theta,',num2str(i*Nts+offset-1),',T))^2;']);
       V = V + tmp;

       if filter
           eval(['tmp_dot = (y_dot',num2str(i),' - phi_fun_dot(x0,theta,',num2str(i*Nts+offset-1),',T))^2;']);
           V = V + tmp_dot;
       end
    end
    if filter
        V_fun = symfun(V,[x0,theta,T,y,y_dot]);
    else
        V_fun = symfun(V,[x0,theta,T,y]);
    end

    % compute gradient
    V_grad = gradient_sym(V,state);
    V_grad = simplify(V_grad);
    if filter
        V_grad_fun = symfun(V_grad,[x0,theta,T,y,y_dot]);
    else
        V_grad_fun = symfun(V_grad,[x0,theta,T,y]);
    end

    % hessian sym
    V_hess = gradient_sym(V_grad,state);
    V_hess = simplify(V_hess);
    if filter
        V_hess_fun = symfun(V_hess,[x0,theta,T,y,y_dot]);
    else
        V_hess_fun = symfun(V_hess,[x0,theta,T,y]);
    end
    
    F = {};
    F_tot = cell(size(V_hess,1),1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % compute V
    for a=1:length(x_grid(1).val)
        for b=1:length(x_grid(2).val)
            tmp_str = ['V_val(',num2str(b),',',num2str(a),')= double(V_fun(x_grid(1).val(',num2str(a),'),x_grid(2).val(',num2str(b),'),T_true'];
            for i=1:Nw
               tmp_str = [tmp_str,',y_star(',num2str(i),')'];  
            end
            if filter
               for i=1:Nw
                   tmp_str = [tmp_str,',y_star_dot(',num2str(i),')'];  
                end 
            end
            tmp_str = [tmp_str, '));'];
            eval(tmp_str);
        end 
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%% check and compute gradient %%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot V
    for a=1:length(x_grid(1).val)
        for b=1:length(x_grid(2).val)
            tmp_str = ['tmp_grad = double(V_grad_fun(x_grid(1).val(',num2str(a),'),x_grid(2).val(',num2str(b),'),T_true'];
            for i=1:Nw
               tmp_str = [tmp_str,',y_star(',num2str(i),')'];  
            end
            if filter
               for i=1:Nw
                   tmp_str = [tmp_str,',y_star_dot(',num2str(i),')'];  
                end 
            end
            tmp_str = [tmp_str, '));'];
            eval(tmp_str);
            V_grad_val(:,a,b) = tmp_grad;
        end 
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot V_hess eig
    tmp_str = 'V_hess_val = V_hess_fun(XVAL,THETA,T_true';
    for i=1:Nw
       tmp_str = [tmp_str,',y_star(',num2str(i),')']; 
    end
    if filter
       for i=1:Nw
           tmp_str = [tmp_str,',y_star_dot(',num2str(i),')'];  
        end 
    end
    tmp_str = [tmp_str, ');'];
    eval(tmp_str);
    for i=1:length(V_hess_val)
        V_hess_val{i} = double(V_hess_val{i});
    end
    

    H_map_sym = zeros(size(V_val));
    H_map_sym_bool = zeros(size(V_val));
    for i=1:length(x_grid(1).val)
       for j=1:length(x_grid(2).val)
           for a=1:length(state)
               for b=1:length(state)
                   tmp_V(a,b) = double(V_hess_val{a,b}(j,i));
               end
           end
           V_hess_matrix{j,i} = tmp_V;
           tmp_eig = eig(tmp_V);
           IsPositiveDefinite = ~(any(tmp_eig<1e-3));
           if IsPositiveDefinite
                H_map_sym(j,i) = norm(tmp_eig);
                H_map_sym_bool(j,i) = 1;
           else
                H_map_sym(j,i) = -norm(tmp_eig);
                H_map_sym_bool(j,i) = NaN;
           end
       end
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

