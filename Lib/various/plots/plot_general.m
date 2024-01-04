%% plot_general
% file: plot_general.m
% author: Federico Oliva
% date: 20/12/2023
% description: plot the results of the simulation (simulation_general)
% INPUT: 
%           obs: obsopt instance
% OUTPUT: none
function plot_general(obs)
    
    % all the figures will be in a single dock
    set(0,'DefaultFigureWindowStyle','docked');            
    
    % fontsize of the plots
    fontsize = 15;

    % linewidth of the plots
    Lwidth = 1.5;

    % counter of the figures
    fig_count = 0;
    
    %%% plot state estimation
    % increase the fig counter
    fig_count = fig_count+1;

    % open a figure
    figure(fig_count)  

    % set title
    sgtitle('State estimation')

    % cycle over VarsPlot
    for i=1:length(obs.Params.VarsPlot)

        % open subplot
        subplot(length(obs.Params.VarsPlot),1,i);

        % layout
        hold on
        grid on
        box on
        
        % cycle over trajectories
        for traj=1:obs.Params.Ntraj
            
            try

                % try to plot the nominal state
                plot(obs.T,obs.X(traj).val(obs.Params.VarsPlot(i),:),'b--','LineWidth',Lwidth);

                % set a flag (used in the legend)
                nominal_state = 1;

            catch

                % there might be no one
                warning('No nominal state present')

                % set a flag (used in the legend)
                nominal_state = 0;

            end

            % plot estimated state
            plot(obs.T,obs.Xhat(traj).val(obs.Params.VarsPlot(i),:),'r--','LineWidth',Lwidth);                                                  
        end    

        % set font size
        set(gca,'fontsize', fontsize) 

        % set y label
        ylabel(['x_',num2str(obs.Params.VarsPlot(i))])

    end

    % set label on x axis
    xlabel('time [s]')

    % legend
    if nominal_state
        legend('True','Est')
    else
        legend('Est')
    end
        

    %%% plot parameters estimation
    % check if there are parameters (Theta could be empty)
    if ~isempty(obs.Params.ParamsPlot)            

        % increase the figure counter
        fig_count = fig_count+1;

        % open new figure
        figure(fig_count)

        % set title
        sgtitle('Parameters estimation')

        % cycle over ParamsPlot
        for i=1:length(obs.Params.ParamsPlot)

            % open subplot
            subplot(length(obs.Params.ParamsPlot),1,i);

            % layout
            hold on
            grid on
            box on

            % cycle over trajectories
            for traj=1:obs.Params.Ntraj

                try

                    % try to plot the nominal parameters
                    plot(obs.T,obs.X(traj).val(obs.Params.ParamsPlot(i),:),'b--','LineWidth',Lwidth);

                    % set a flag (used in the legend)
                    nominal_state = 1;

                catch

                    % there might be no one
                    warning('No nominal Theta present')

                    % set a flag (used in the legend)
                    nominal_state = 0;

                end

                % plot estimated parameter
                plot(obs.T,obs.Xhat(traj).val(obs.Params.ParamsPlot(i),:),'r--','LineWidth',Lwidth);  

            end

            % set fontsize 
            set(gca,'fontsize', fontsize)      

            % label on y axis
            ylabel(['x_',num2str(obs.Params.ParamsPlot(i))])

        end

        % label on x axis
        xlabel('time [s]')

        % legend
        if nominal_state
            legend('True','Est')
        else
            legend('Est')
        end
        
    end
    
    %%% plot measurements       
    % increase figure counter
    fig_count = fig_count+1;

    % open new figure
    figure(fig_count)

    % set title
    sgtitle('Sampled measured')

    % define axes handles
    ax = zeros(1,3);

    % number fo subplots depending on the output dimension
    n_subplot = obs.Params.DimOut;

    % create subplots
    subplot(n_subplot,1,1);

    % cycle over the output 
    for k=1:obs.Params.DimOut
        
        % indicize axes
        ax_index = k;
        ax(ax_index)=subplot(n_subplot,1,ax_index);
        
       % layout 
        grid on
        hold on
        box on
        
        % optimizing instants
        WindowTimeOptSamples = obs.Params.OptTime;
        WindowTimeOpt = obs.T(WindowTimeOptSamples);

        % selected optimization instants
        WindowTimeSelSamples = obs.Params.SelectTime;
        WindowTimeSel = obs.T(WindowTimeSelSamples);
        
        % cycle over trajectories
        for traj=1:obs.Params.Ntraj

            % plot measured values
            plot(obs.T,obs.Y(traj).val(k,:),'b--','LineWidth',Lwidth)

            % plot estimated values
            plot(obs.T,obs.Yhat(traj).val(k,:),'r--','LineWidth',Lwidth)

            % plot sampled values
            plot(WindowTimeOpt,obs.Y(traj).val(k,WindowTimeOptSamples),'bo','MarkerSize',5);

            % plot accepted optimizations
            plot(WindowTimeSel,obs.Y(traj).val(k,WindowTimeSelSamples),'ms','MarkerSize',15);
                     
        end

        % set fontsize
        set(gca,'fontsize', fontsize)

        % label on y axis
        ylabel(strcat('y_',num2str(k)));     

    end

    % label on x axis
    xlabel('simulation time [s]');

    % legend
    legend('meas','sampled')
    linkaxes(ax(1:n_subplot),'x');
    
end