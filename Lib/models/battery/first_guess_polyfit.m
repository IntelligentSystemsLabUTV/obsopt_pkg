%% main script

% first defines
save('obs_default','obs');

% set time
t_pos = 1;
traj = 1;

% define param
Nparam = 4;
Npoly = 4;

% opt vars
compare_vars = 3:6;
update_vars = [ 7 11 15 19 23;...
                8 12 16 20 24;...
                9 13 17 21 25;...
                10 14 18 22 26];

% init est
old_Xest = obs.init.X_est(traj).val(:,1);

% data to fit
X = obs.init.params.input_soc';
Y = [obs.init.params.input_OCV'; ...
     obs.init.params.input_R0'; ...
     obs.init.params.input_R1'; ...
     obs.init.params.input_C1'];

for param = 1:Nparam    

    % solve the optimisation problem
    [tmp, S] = polyfit(X,Y(param,:),Npoly);
    
    old_Xest(update_vars(param,:)) = tmp; 
end

% restore obs
load obs_default

% obs.init
obs.init.cloud_X = X;
obs.init.cloud_Y = Y;

% update params - init
update_vars_row = reshape(update_vars,1,size(update_vars,1)*size(update_vars,2));

% if DT
obs.init.X_est(traj).val(update_vars_row,1) = old_Xest(update_vars_row);

% update est params
obs.init.params = obs.setup.params.params_update(obs.init.params,obs.init.X_est(traj).val(:,1));

% update true vals
obs.init.X(1).val(update_vars_row,1) = obs.init.X_est(1).val(update_vars_row,1);
obs.setup.params = obs.setup.params.params_update(obs.init.params,obs.init.X(traj).val(:,1));

% clear workspace
delete('obs_default.mat')