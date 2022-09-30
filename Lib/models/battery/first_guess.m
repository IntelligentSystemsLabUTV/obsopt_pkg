%% main script

% first defines
save('obs_default','obs');

% set time
t_pos = 1;
traj = 1;

% set Npoint for cloud
rng default
Npoint = 100;

% define noise
perc = 1;
sigma = 0*[0.05 0.2 0.2 0.2];

% define param
Nparam = 4;
Npoly = 4;
CT = 0;

% opt vars
opt_vars = [7:10,11:14,15:18,19:22,23:26,27:30];
compare_vars = 3:6;
update_vars = [ 7 11 15 19 23 27;...
                8 12 16 20 24 28;...
                9 13 17 21 25 29;...
                10 14 18 22 26 30];

% optimset
myoptioptions = optimset('MaxIter', 300, 'display','off', 'MaxFunEvals',Inf,'TolFun',0,'TolX',0); 

% generate dataset
x_start = obs.init.X(traj).val(:,1);
x_out = gen_meas(obs,x_start,Npoint,perc,sigma);

% init est
old_Xest = obs.init.X_est(traj).val(:,1);

for param = 1:Nparam
    % opt vars
    obs.setup.opt_vars = opt_vars(param + (0:Npoly-1)*Nparam);
    obs.setup.compare_vars = compare_vars(param);
    
    % set the not optimised vars
    tmp = 1:length(obs.init.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(obs.setup.opt_vars)
        tmp_idx = intersect(tmp_idx,find(tmp~=obs.setup.opt_vars(i)));
    end
    obs.setup.nonopt_vars = tmp_idx;
    
    % evolute for tspan

    obs.init.traj = traj;

    x0 = obs.init.X_est(traj).val(:,1);
    x0_nonopt = x_out(obs.setup.nonopt_vars,:);
    x0_opt = x0(obs.setup.opt_vars);

    % solve the optimisation problem
%     tmp = fminsearch(@(x)cost_function(obs,x,x0_nonopt,x_out(obs.setup.compare_vars,:),1), x0_opt, myoptioptions); 
    tmp = polyfit(x_out(1,:),x_out(obs.setup.compare_vars,:),Npoly-1);
    
    old_Xest(update_vars(param,1:(Npoly))) = flip(tmp); 
end

% restore obs
load obs_default

% obs.init
obs.init.cloud_Y = x_out(3:6,:);
obs.init.cloud_X = x_out(1,:);

% update params - init
update_vars_row = reshape(update_vars(:,1:(Npoly)),1,size(update_vars(:,1:(Npoly)),1)*size(update_vars(:,1:(Npoly)),2));

% if DT
if ~CT
    obs.init.X_est(traj).val(update_vars_row,1) = old_Xest(update_vars_row);
else
    % if CT
    obs.init.X_est(traj).val(update_vars_row-Nparam,1) = old_Xest(update_vars_row);
end

% update est params
obs.init.params = obs.setup.params.params_update(obs.init.params,obs.init.X_est(traj).val(:,1));

% update true vals
obs.init.X(1).val(update_vars_row,1) = obs.init.X_est(1).val(update_vars_row,1);
obs.setup.params = obs.setup.params.params_update(obs.init.params,obs.init.X(traj).val(:,1));

% clear workspace
delete('obs_default.mat')

%%
function x_out = gen_meas(obs_default,x,Npoint,perc,sigma)    

    % params
    params = obs_default.init.params;
    
    % initialize state
    x_out = zeros(length(x),Npoint);
    
    % create cloud
    for i=1:Npoint
        
        % generate SOC        
        x_out(1,i) =  unifrnd(max(0,x(1)-perc),min(1,x(1)+perc));
        
        % compute the real val
        ref(3) = spline(params.input_soc, params.input_OCV, x_out(1,i));
        ref(4) = spline(params.input_soc, params.input_R0, x_out(1,i));
        ref(5) = spline(params.input_soc, params.input_R1, x_out(1,i));
        ref(6) = spline(params.input_soc, params.input_C1, x_out(1,i));
        
        % compute and perturb params
        x_out(3,i) = spline(params.input_soc, params.input_OCV, x_out(1,i)) + sigma(1)*ref(3)*randn(1);
        x_out(4,i) = spline(params.input_soc, params.input_R0, x_out(1,i)) + sigma(2)*ref(4)*randn(1);
        x_out(5,i) = spline(params.input_soc, params.input_R1, x_out(1,i)) + sigma(3)*ref(5)*randn(1);
        x_out(6,i) = spline(params.input_soc, params.input_C1, x_out(1,i)) + sigma(4)*ref(6)*randn(1);
       
    end

end

%%
function J = cost_function(varargin)

    % get stuff
    obs_default = varargin{1};
    
    % get opt state
    x_opt = varargin{2};

    % non optimised vals
    x_nonopt = varargin{3};
    
    % build the state
    x(obs_default.setup.opt_vars,1) = x_opt;    
    
    % get target
    y = varargin{4};
    
    % get time pos
    t_pos = varargin{5};
    
    % add stuff to obs
    obs_default.init.Y_full_story(obs_default.init.traj).val(:,:,1) = 1;         
    
    % get cost function
    for i=1:size(y,2)
        
        % get the non optimised vars
        x(obs_default.setup.nonopt_vars,1) = x_nonopt(:,i);
        
        % update params
        obs_default.init.params = obs_default.setup.params.params_update(obs_default.init.params,x); 
        
        % compute the model
        yhat(:,i) = model_battery_tushar(t_pos,x,obs_default.init.params,obs_default);
                
    end
    
    if 0 && (~obs_default.init.normalised)         
        for i=1:length(obs_default.setup.compare_vars)
            E = (y(i,:) - yhat(obs_default.setup.compare_vars(i),:)).^2;        
            Emax = max(E);
            if Emax == 0
                Emax = 1;
            end                
%             obs_default.init.scale_factor_scaled(i) = norm(y(obs_default.setup.compare_vars(i),:))/Emax;
            obs_default.init.scale_factor_scaled(i) = 1/Emax;
        end
        obs_default.init.normalised = 1;    
    else
        obs_default.init.scale_factor_scaled = ones(1,length(obs_default.setup.compare_vars));
    end
    
    % update the cost function
    statediff = y-yhat(obs_default.setup.compare_vars,:);
    W = obs_default.init.scale_factor_scaled;
    for i=1:length(obs_default.setup.compare_vars)
        J(i) = statediff(i,:)*W(i)*transpose(statediff(i,:));
    end
    
    J = sum(J);

end