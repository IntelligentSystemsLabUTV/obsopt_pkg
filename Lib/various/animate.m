%% animate double pendulum
function dist = animate(params,obs,play)

    x = obs.init.X;
    x_1 = @(t) params.Lt1*cos(x(1,t));
    y_1 = @(t) params.Lt1*sin(x(1,t));
    x_2 = @(t) params.Lt1*cos(x(1,t))+params.Lt2*cos(x(2,t));
    y_2 = @(t) params.Lt1*sin(x(1,t))+params.Lt2*sin(x(2,t));
    dist_10 = @(t) norm([x_1(t),y_1(t)]);
    dist_12 = @(t) norm([x_1(t)-x_2(t),y_1(t)-y_2(t)]);
    
%     endIter = 1;
    endIter = obs.setup.Niter;
    
    if play
        fanimator(@(t) plot(x_1(t),y_1(t),'ro','MarkerSize',params.M1*10,'MarkerFaceColor','r'),'AnimationRange', [1, endIter], 'FrameRate', 1);
        axis equal
        hold on;
        grid on
        fanimator(@(t) plot([0 x_1(t)],[0 y_1(t)],'r-'),'AnimationRange', [1, endIter], 'FrameRate', 1);
        fanimator(@(t) plot(x_2(t),y_2(t),'go','MarkerSize',params.M2*10,'MarkerFaceColor','g'),'AnimationRange', [1, endIter], 'FrameRate', 1);
        fanimator(@(t) plot([x_1(t) x_2(t)],[y_1(t) y_2(t)],'g-'),'AnimationRange', [1, endIter], 'FrameRate', 1);
        fanimator(@(t) text(-0.3,-0.3,"Timer: "+num2str(t,2)),'AnimationRange', [1, endIter], 'FrameRate', 1);
        hold off;
        playAnimation('SpeedFactor', 1);
    end
    
    %% compute distance
    dist = zeros(2,obs.setup.Niter);
    for i=1:obs.setup.Niter
        dist(1,i) = dist_10(i);
        dist(2,i) = dist_12(i);
    end
    
end