%%
function plot_VolterraLotka(obs,control)

    set(0,'DefaultFigureWindowStyle','docked');   

    % states
    figure()
    hold on
    grid on
    plot(obs.setup.time,obs.init.X(1).val(1,:),'LineWidth',2)
    plot(obs.setup.time,obs.init.X(1).val(2,:),'LineWidth',2)
    if control
        set(gca,'ColorOrderIndex',1)
        plot(obs.setup.time(1:end-1),obs.init.input_story_ref.val(1,:),'--','LineWidth',2)
        plot(obs.setup.time(1:end-1),obs.init.input_story_ref.val(2,:),'--','LineWidth',2)
    end
    set(gca,'fontsize', 20)  
    if ~control
        legend('G','L')
    else
        legend('G','L','u_G','u_L')
    end
    xlabel('time')
    ylabel('animals')

    % phase
    figure()
    hold on
    grid on
    plot(obs.init.X(1).val(1,1),obs.init.X(1).val(2,1),'ro','LineWidth',2,'MarkerSize',5);   
    plot(obs.init.X(1).val(1,:),obs.init.X(1).val(2,:),'b','LineWidth',2)   
    plot(obs.init.X(1).val(1,end),obs.init.X(1).val(2,end),'ko','LineWidth',2,'MarkerSize',5);   
    set(gca,'fontsize', 20)  
    xlabel('G')
    ylabel('L')


end