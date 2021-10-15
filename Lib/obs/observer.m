%% attitude observer - optimisation algorithm
function obj = observer(obj,xhat,y,params)

    yhat = obj.setup.measure(xhat,params);

    %%%%%%%%%%%%%%%%%%%%% INSERT OPT %%%%%%%%%%%%%%%%%%%%%
    obj.init.Y_full_story(:,:,end+1) = y;
    obj.init.Yhat_full_story(:,:,end+1) = yhat;

    % fisrt bunch of data - read Y every Nts and check if the signal is
    distance = obj.init.ActualTimeIndex-obj.init.Y_space(end);
    if  (distance >= obj.setup.Nts)

        if obj.setup.print
            % Display iteration slengthtep
            disp(['n window: ', num2str(obj.setup.w),'  n samples: ', num2str(obj.setup.Nts)])
            disp(['Last cost function: ', num2str(obj.init.Jstory(end))]);
            disp(['N. optimisations RUN: ',num2str(obj.init.opt_counter)]);
            disp(['N. optimisations SELECTED: ',num2str(obj.init.select_counter)]);
        end

        %%%% OUTPUT measurements - buffer of w elements
        % measures
        obj.init.Y(:,:,1:end-1) = obj.init.Y(:,:,2:end);
        obj.init.Y(:,:,end) = y;

        % backup
        Y_space_backup = obj.init.Y_space;
        Y_space_full_story_backup = obj.init.Y_space_full_story;

        % adaptive sampling
        obj.init.Y_space(1:end-1) = obj.init.Y_space(2:end);
        obj.init.Y_space(end) = obj.init.ActualTimeIndex;
        obj.init.Y_space_full_story(end+1) = obj.init.ActualTimeIndex;

        % store measure times
        obj.init.temp_time = [obj.init.temp_time obj.init.ActualTimeIndex];

        cols_nonzeros = length(find(sum(obj.init.Y ~= 0,1)));

        if cols_nonzeros >= obj.setup.w

            if obj.setup.forward

                %%%% FLUSH THE BUFFER IF SAFETY FLAG %%%%
                first_nonzero = find(obj.init.Y_space,1,'first');
                Y_space_nonzero = obj.init.Y_space(first_nonzero:end);
                max_dist = max(diff(Y_space_nonzero));
                if isempty(max_dist)
                    max_dist = 1;
                end
                n_samples = min(length(obj.init.Y_space_full_story)-1,obj.setup.w);
                buf_Y_space_full_story = obj.init.Y_space_full_story(end-n_samples:end);

                % back time index
                buf_dist = diff(buf_Y_space_full_story);
                obj.init.BackTimeIndex = obj.init.time(max(obj.init.ActualTimeIndex-sum(buf_dist),1)); 
                obj.init.BackIterIndex = find(obj.init.time==obj.init.BackTimeIndex);

                % set of initial conditions
                obj.init.temp_x0 = obj.init.X_est(:,obj.init.BackIterIndex);

                % Optimisation

                % Optimisation (only if distance_safe_flag == 1)
                opt_time = tic;
                % save J before the optimisation to confront it later
                J_before = obj.cost_function(obj.init.temp_x0,'params',params);


                %%%%% OPTIMISATION - NORMAL MODE %%%%%%
                [NewXopt, J, obj.init.exitflag] = obj.setup.fmin(@(x)obj.cost_function(x,'params',params),obj.init.temp_x0,obj.init.myoptioptions);

                % opt counter
                obj.init.opt_counter = obj.init.opt_counter + 1;

                % adaptive buffer backup restore
                obj.init.Y_space = Y_space_backup;
                obj.init.Y_space_full_story = Y_space_full_story_backup;

                % check J_dot condition
                J_diff = (J/J_before);

                if J_diff <= obj.setup.Jdot_thresh

                    % update
                    obj.init.X_est(:,obj.init.BackIterIndex) = NewXopt;

                    % store measure times
                    obj.init.opt_chosen_time = [obj.init.opt_chosen_time obj.init.ActualTimeIndex];

                    % counters
                    obj.init.jump_flag = 0;
                    obj.init.select_counter = obj.init.select_counter + 1;

                    x_propagate = NewXopt;

                    %%%%%%%%%%%%%%%%% FIRST MEASURE UPDATE %%%%%%%%
                    % manage measurements
                    % set the derivative buffer as before the optimisation process (multiple f computation)
                    back_time = obj.init.BackIterIndex;

                    %%%% ESTIMATED measurements
                    % measures       
                    % NB: the output storage has to be done in
                    % back_time+1 as the propagation has been
                    % performed 
                    Yhat = obj.setup.measure(x_propagate,params);
                    obj.init.Yhat_full_story(:,back_time+1) = Yhat;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

                    %%%%%%%%%%% PROPAGATION %%%%%%%%%%%%%%%%%%%%%%%
                    n_iter_propagate = obj.init.Niter-obj.init.BackIterIndex;

                    for j =1:n_iter_propagate 
                        % back time
                        back_time = obj.init.BackIterIndex+j;

                        % integrate
                        X = obj.setup.ode(@(t,x)obj.setup.model(back_time, x_propagate, params), params.tspan, x_propagate);
                        x_propagate = X.y(:,end);                      
                        obj.init.X_est(:,back_time) = x_propagate;

                        %%%% ESTIMATED measurements
                        % measures       
                        % NB: the output storage has to be done in
                        % back_time+1 as the propagation has been
                        % performed 
                        Yhat = obj.setup.measure(x_propagate,params);
                        obj.init.Yhat_full_story(:,:,back_time+1) = Yhat;
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
                    obj.init.Jstory(1,end+1) = J;
                else
                    % keep the initial guess
                    obj.init.X_est(:,obj.init.BackIterIndex) = obj.init.temp_x0;
                end                        

                % adaptive sampling
                obj.init.Y_space(1:end-1) = obj.init.Y_space(2:end);
                obj.init.Y_space(end) = obj.init.ActualTimeIndex;
                obj.init.Y_space_full_story(end+1) = obj.init.ActualTimeIndex;

                % stop time counter
                obj.init.opt_time(end+1) = toc(opt_time);

            end

        end

        if obj.setup.print
            clc;
        end

    end

        
end
