%% control drive function (observer or control design)
function drive_out = drive(varargin) 

    obj = varargin{1};   

    % init drive
    drive_out = [];

    % x - varargin
    x_star = varargin{2}(1:obj.init.params.dim_state,:);            
    x = varargin{3}(1:obj.init.params.dim_state,:);      


    % just x
%     drive_out = [drive_out; x(obj.setup.plot_vars)];
    drive_out = [drive_out; x];

    % get y    
    if 1    
    y_meas = varargin{4}(:,:,end);
    pos = varargin{5};
    
    y = obj.setup.measure(x,obj.init.params,pos,obj.init.input_story(obj.init.traj).val(:,max(1,pos-1)));    
    
    %%% compute filters %%%
    y_filt = [];
    y_filt_meas = [];
    if 1 && obj.setup.Nfilt > 0                     

        % how long iterate        
        tspan_pos = [max(1,pos-1), pos];                
        
        % update buffer - only first (RK4)
        obj.init.Y_buffer_control(obj.init.traj).val(1,:,pos) = y;                        
        
        try
            ok = 1;
            [Y_filt, x_filter] = obj.measure_filter(obj.init.Y_buffer_control(obj.init.traj).val,tspan_pos,obj.init.X_filter_buffer_control(obj.init.traj));
        catch            
            ok=0;
        end

        if ok
            for filt=1:obj.setup.Nfilt
                for dim=1:obj.setup.dim_out
                    % Lsim
                    y_filt(filt,dim) = Y_filt{filt,dim}.val(end);
                    obj.init.X_filter_buffer_control(obj.init.traj).val{filt,dim}(:,unique(tspan_pos)) = x_filter{filt,dim}.val;
                                        
                end
                
                % update buffer - only first (RK4)
                obj.init.Y_buffer_control(obj.init.traj).val(1+filt,:,pos) = y_filt(filt,:);
            end
            
            y_filt = reshape(y_filt,size(y));
            y_pad = zeros(size(y).*[2,1]);
            y_pad(1:2:end) = y;
            y_pad(2:2:end) = y_filt;
            y = y_pad;
            
        else
%             y_meas = reshape(y_meas,size(y_meas,1)*size(y_meas,2));
            y = y_meas;            
        end
    end
          
        y_meas = reshape(y_meas,size(y));
        tmp = y_meas-y;        
        tmp = reshape(tmp,length(y),1);

        drive_out = [drive_out; tmp];
    end
    
    % save drive story
    obj.init.drive_out(obj.init.traj).val(:,pos) = drive_out;

end