function xhat = wrapper_observer(t,x,y,Time,obs) 
%#codegen

    % get time index
    tdiff = Time-t;   
    pos = find(abs(tdiff) == min(abs(tdiff)),1,'first'); 
    pos = min(pos,length(Time));

    % set time index
    obs.init.ActualTimeIndex = pos;
    
    % set output
    y_meas.val = y;
    
    % call the observer
    obs = obs.observer(x,y_meas);
    
    xhat = obs.init.X_est.val(:,pos);
end