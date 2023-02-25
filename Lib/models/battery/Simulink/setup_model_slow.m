%% setup data for simulink model
function [obs, params, SimParams] = setup_model_slow(tstart,tend,Ts)

    % set initial and final time instant
    t0 = tstart;
    
    %%%% params init function %%%%
    params_init = @params_battery_tushar;    
    %%%% params update function %%%%
    params_update = @params_update_battery_tushar;    
    %%%% model function %%%%
    model = @model_battery_tushar;            
    %%%% measure function %%%%
    measure = @measure_battery_tushar;
    % measure = @measure_general;    
    
    % MHE
    Nw = 30;
    Nts = 20;
%     Nts = [1*ones(1,5) 20*ones(1,25)];
    
    %%%% filters %%%%    
    [filter, filterScale, ~] = filter_define(Ts,Nts);
    
    %%%% integration method %%%%
    ode = @odeDD;
    
    %%%% input law %%%
    input_law = @control_battery;        
    
    % setup params
    params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0, 'params_update', params_update, ...
            'model',model,'measure',measure,'ObservedState',[1],'ode',ode, 'odeset', [1e-3 1e-6], ...
            'input_enable',1,'input_law',input_law,'params_init',params_init);
        
    %%%% observer init %%%%    
    terminal_states = params.opt_vars;
    terminal_weights = 1e0*ones(size(terminal_states));
    % all state
    terminal_weights(2) = 1e-1;
    terminal_weights(3:5) = 5e0;
    terminal_weights(6:8) = 4e0;
    terminal_weights(9:11) = 3e0;    
    terminal_weights(12:14) = 2e0;
%     terminal_weights = 1e2*terminal_weights;
    
    obs = obsopt('DataType', 'measured', 'optimise', 1, 'GlobalSearch', 0, 'MultiStart', 0, 'J_normalise', 1, 'MaxOptTime', Inf, ... 
          'Nw', Nw, 'Nts', Nts, 'ode', ode, 'PE_maxiter', 0, 'WaitAllBuffer', 0, 'params',params, 'filters', filterScale,'filterTF', filter, ...
<<<<<<< HEAD
          'Jdot_thresh',0.99,'MaxIter',50, 'Jterm_store', 0, 'AlwaysOpt', 1 , 'print', 0 , 'SafetyDensity', 5, 'AdaptiveHist', [1*1e-3, 1*1e-3, 1.3], 'PEPos', [1 1], ...
          'AdaptiveSampling',0, 'FlushBuffer', 0, 'opt', @fminsearchcon, 'terminal', 1, 'terminal_states', terminal_states, 'terminal_weights', terminal_weights, 'terminal_normalise', 1, ...
=======
          'Jdot_thresh',0.99,'MaxIter',1000, 'Jterm_store', 0, 'AlwaysOpt', 1 , 'print', 0 , 'SafetyDensity', 5, 'AdaptiveHist', [1*1e-3, 1*1e-3, 1.3], 'PEPos', [1 1], ...
          'AdaptiveSampling',1, 'FlushBuffer', 0, 'opt', @fminsearchcon, 'terminal', 1, 'terminal_states', terminal_states, 'terminal_weights', terminal_weights, 'terminal_normalise', 1, ...
>>>>>>> 6e2e2db7e18616e4f22ee99cda5d0b40ae9c46b8
          'ConPos', [1], 'LBcon', [0], 'UBcon', [1], 'Bounds', 1, 'BoundsPos',[4 5], 'BoundsValLow',[5e-3 5e-3], 'BoundsValUp',[Inf Inf], 'BoundsWeight',[1 1]);
      
    % update vars for multi MHE
    obs.setup.update_vars = [1:2 8:10];
      
    %%%%%%%%%% SIMULINK STUFF %%%%%%%%%
    % load OCV and R data (for ECM in simulink)
    input_data = params.input_data;
    params.input_time = input_data.Time;
    params.input_current = input_data.Current;
    params.input_soc = reshape(input_data.SOC(2:end),length(input_data.SOC(2:end)),1);
    params.LengthFit = length(input_data.SOC(2:end));
    % model    
    params.input_OCV = reshape(input_data.OCV(2:end),length(input_data.OCV(2:end)),1);    
    params.input_R0 = reshape(input_data.R0(2:end),length(input_data.R0(2:end)),1);
    params.input_R1 = reshape(input_data.R1(2:end),length(input_data.R1(2:end)),1);
    params.input_C1 = reshape(input_data.C1(2:end),length(input_data.C1(2:end)),1);    
    % nominal
    params.input_OCV_nominal = reshape(input_data.OCV_nominal(2:end),length(input_data.OCV(2:end)),1);    
    params.input_R0_nominal = reshape(input_data.R0_nominal(2:end),length(input_data.R0(2:end)),1);
    params.input_R1_nominal = reshape(input_data.R1_nominal(2:end),length(input_data.R1(2:end)),1);
    params.input_C1_nominal = reshape(input_data.C1_nominal(2:end),length(input_data.C1(2:end)),1);    

    % set model temperature
    params.Temperature = 313.15;
    params.Tend = tend;

    % propagation model in simulink
    params.model_SIM = @model_battery_tushar_reference_SIM;
    
    % create the bus to be provided to simulink
    stack = [params.C_n; params.eta; params.input_soc; params.input_OCV; params.input_R0; params.input_R1; params.input_C1];
    SimParams = timeseries(stack.*ones(size(params.time)),params.time);
    
end