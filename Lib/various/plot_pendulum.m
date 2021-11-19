%% function to plot the pendulum
function plot_pendulum(x,params)
    
    % get params
    single_pendulum = isempty(intersect(fieldnames(params),'Lt2'));
    
    figure()
    hold on
    box on
    grid on
    
    % origin
    plot(0,0,'r*');
    
    % dynamics
    if single_pendulum
        x_tmp = params.Lt1*cos(x(1,:));
        y_tmp = params.Lt1*sin(x(1,:));
        plot(x_tmp,y_tmp,'bo');
    else
        x_tmp = params.Lt1*cos(x(1,:));
        y_tmp = params.Lt1*sin(x(1,:));
        plot(x_tmp,y_tmp,'bo');
        
        x_tmp = params.Lt1*cos(x(1,:)) + params.Lt2*cos(x(2,:));
        y_tmp = params.Lt1*sin(x(1,:)) + params.Lt2*sin(x(2,:));
        plot(x_tmp,y_tmp,'go');
    end

end