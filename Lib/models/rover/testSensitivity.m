%% 
function testSensitivity(obs,c,num)
    params = obs.init.params;
    for i=1:params.Nanchor
        dhdx(i,:) = nonzeros(squeeze(obs.init.GHx(:,i,1)));
        dhdy(i,:) = nonzeros(squeeze(obs.init.GHx(:,i,6)));
        dhdz(i,:) = nonzeros(squeeze(obs.init.GHx(:,i,11)));
    end   
    
    % plot
    figure(num)
    subplot(3,1,1)
    hold on
    grid on
    for i=1:params.Nanchor
    plot(abs(dhdx(i,:)),c);
    end
    subplot(3,1,2)
    hold on
    grid on
    for i=1:params.Nanchor
    plot(abs(dhdy(i,:)),c);
    end
    subplot(3,1,3)
    hold on
    grid on
    for i=1:params.Nanchor
    plot(abs(dhdz(i,:)),c);
    end
end