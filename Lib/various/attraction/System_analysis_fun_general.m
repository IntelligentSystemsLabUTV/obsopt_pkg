%% script for the theoretical test
function out = System_analysis_fun_general(x0,offset,filter,Nw,Nts,Nsamples)

    % flags 
    out.numeric = 1;
    out.symbolic = 0;

    % set sampling time
    Ts = 5e-2;

    % set initial and final time instant
    t0 = 0;
    tend = (Nts*Nw+offset+1)*Ts;

    params_init = @params_mockup_chaos;
    model = @model_mockup_chaos;
    measure = @measure_general;
    ode = @oderk4;

    % define observer
    params = model_init('Ts',Ts,'T0',[t0, tend],'noise',0,'noise_spec',[0, 0],...
            'model',model,'measure',measure,'StateDim',2,'ObservedState',[1],'ode',ode,...
            'input_enable',0,'dim_input',1,'input_law',[],'params_init',params_init);
        
    % define init condition
    params.X = x0;

    % create observer class instance. For more information on the setup
    % options check directly the class constructor
    obs = obsopt_general_adaptive_flush('Nw',Nw,'Nts',Nts,'ode',ode, 'optimise', 0, ...    
          'params',params, 'filters',[1,filter,0,0],'Jdot_thresh',0.9,'MaxIter',200,...
          'AlwaysOpt',0,'print',0,'SafetyDensity',5,'AdaptiveHist',[5e-3, 1e-2],...
          'AdaptiveSampling',0, 'FlushBuffer', 1, 'Jterm_store',0, 'opt', @fminsearch);
      
    
    
    % hessian threshold
    thresh_eig = 0;

    % define evaluation grid
    x_range = [0.6*params.X, params.X, 1.4*params.X];
    for i=1:params.StateDim
        left = linspace(x_range(i,1),x_range(i,2),Nsamples(i));
        right = linspace(x_range(i,2),x_range(i,3),Nsamples(i));
        x_grid(i).val = [left, right(2:end)];
        Tgran(i).val = diff(x_grid(i).val);
    end
    
    tmp_str = '[';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'X(',num2str(i),').mesh,'];
    end
    tmp_str = [tmp_str(1:end-1),'] = meshgrid('];
    for i=1:params.StateDim
        tmp_str = [tmp_str,'x_grid(',num2str(i),').val,'];
    end
    tmp_str = [tmp_str(1:end-1),');'];
    eval(tmp_str);
    
    %%% output
    out.X = X;
    out.x_grid = x_grid;
    out.x_range = x_range;
    out.StateDim = params.StateDim;
    out.Tgran = Tgran;
    out.offset = offset;
    out.Nw = Nw;
    out.Nts = Nts;
    out.Ts = Ts;
    out.filter = filter;
    out.x0 = x0;
    out.thresh_eig = thresh_eig;
    out.border_grad = 1;
    out.border_hess = 2;


    %% NUMERIC ANALYSIS %%
    % compute cost function
    % init cost function
    tmp_str = 'V_num = zeros(';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'length(x_grid(',num2str(i),').val),'];
    end
    tmp_str = [tmp_str(1:end-1), ');'];
    eval(tmp_str);
    
    % for loop
    tmp_str = '';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'for IT',num2str(i),'=1:length(x_grid(',num2str(i),').val)',newline];
    end
    tmp_str = [tmp_str, newline, 'IT = ['];
    for i=1:params.StateDim
       tmp_str = [tmp_str,'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1),'];'];
    tmp_str = [tmp_str, newline, '[obs, V_num, out] = observer_internal_fun(x_grid,params,obs,out,IT,V_num);', newline, newline];
    for i=1:params.StateDim
       tmp_str = [tmp_str,'end', newline]; 
    end
    eval(tmp_str);

    % gradient analysis
    tmp_str = '[';
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'GV{',num2str(i),'},']; 
    end
    tmp_str = [tmp_str(1:end-1), '] = gradient(V_num,'];
    for i=1:params.StateDim
        tmp_str = [tmp_str, 'Tgran(',num2str(i),').val(1),'];
    end
    tmp_str = [tmp_str(1:end-1), ');'];
    eval(tmp_str); 

    % hessian analysis
    for i=1:params.StateDim
        tmp_str = '[';
        for j=1:params.StateDim
            tmp_str = [tmp_str, 'HV{',num2str(i),',',num2str(j),'},']; 
        end
        tmp_str = [tmp_str(1:end-1), '] = gradient(GV{',num2str(i),'},'];
        for j=1:params.StateDim
            tmp_str = [tmp_str, 'Tgran(',num2str(j),').val(1),'];
        end
        tmp_str = [tmp_str(1:end-1), ');'];
        eval(tmp_str); 
    end

    % region of attraction
    H_map = zeros(size(V_num));
    H_map_bool = zeros(size(V_num));
    
    % for loop
    tmp_str = '';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'for IT',num2str(i),'=1:length(x_grid(',num2str(i),').val)',newline];
    end

    tmp_str = [tmp_str, 'for i=1:params.StateDim', newline, 'for j=1:params.StateDim', newline];
    tmp_str = [tmp_str, newline, 'Hpoint(i,j) = HV{i,j}('];
    
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    
    tmp_str = [tmp_str(1:end-1), ');',newline, newline];  
    tmp_str = [tmp_str, 'end', newline, 'end', newline]; 
    
    tmp_str = [tmp_str, newline, 'tmp_eig=eig(Hpoint);',newline];
    tmp_str = [tmp_str, 'IsPositiveDefinite = ~(any(tmp_eig<thresh_eig));',newline];
    tmp_str = [tmp_str, 'if IsPositiveDefinite', newline];
    % Hmap
    tmp_str = [tmp_str, 'H_map('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = norm(tmp_eig);', newline];
    
    % Hmap bool
    tmp_str = [tmp_str, 'H_map_bool('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = 1;', newline];
    tmp_str = [tmp_str, 'else', newline];
    % Hmap
    tmp_str = [tmp_str, 'H_map('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = -norm(tmp_eig);', newline];
    
    % Hmap bool
    tmp_str = [tmp_str, 'H_map_bool('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = NaN;', newline];
    tmp_str = [tmp_str, 'end', newline,newline];
    
    for i=1:params.StateDim
       tmp_str = [tmp_str,'end', newline]; 
    end
    eval(tmp_str);

    %%% output
    out.V = V_num;
    out.GV = GV;
    out.HV = HV;
    out.Hmap = H_map;
    out.Hmap_bool = H_map_bool;

    H_map_restricted = H_map(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess);
    H_map_bool_restricted = H_map_bool(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess);
    out.attraction_num = length(find(H_map_bool_restricted==1))/length(find(H_map_bool_restricted));
    
end

