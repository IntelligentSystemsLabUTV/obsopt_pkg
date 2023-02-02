%% EKF.m
% file: ekf.m
% author: Federico Oliva
% date: 30/01/2023
% description: EKF observer
% INPUT: obs, state, measure
% OUTPUT: obs

function obs = EKF(obs,xhat_kk_1,y_k)

    % get params
    params = obs.init.params;

    % get time instant
    k = obs.init.ActualTimeIndex;
    tspan = obs.setup.time(max(1,k-1):k);

    % get traj
    traj = obs.init.traj;    

    % get control
    uhat_kk_1 = obs.init.input_story_ref(traj).val(:,k-1);

    % get initial covariance
    sz = size(params.Phat(traj).val);
    Phat_0 = reshape(params.Phat(traj).val(k-1,:,:),sz(2),sz(3));  

    % prediction step (eq. 11)  NB: check if the correct input is used ins.
    X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.setup.params, obs), tspan, xhat_kk_1,params.odeset); 
    xhat_k = X.y(:,end);

    % predicted measure (and store in obs)
    yhat_k = obs.setup.measure(xhat_k,params,tspan(1),uhat_kk_1,obs);
    obs.init.Yhat_full_story(traj).val(1,:,k) = yhat_k;

    % init downsampling
    posH_row = [];    
    yhat_ks = [];
    y_ks = [];

    % get downsampling - CAM
    if mod(k,params.CAM_samp) == 0
        posH_row = [posH_row, [params.pos_p_out params.pos_quat_out]];
        yhat_ks = [yhat_ks; yhat_k([params.pos_p_out params.pos_quat_out])];
        y_ks = [y_ks; y_k([params.pos_p_out params.pos_quat_out])];        
    end

    % get downsampling - IMU
    if mod(k,params.IMU_samp) == 0
        posH_row = [posH_row, [params.pos_acc_out params.pos_omega_out]];
        yhat_ks = [yhat_ks; yhat_k([params.pos_acc_out params.pos_omega_out])];
        y_ks = [y_ks; y_k([params.pos_acc_out params.pos_omega_out])];
    end

    % get jacobians    
    [GFx, GFw, GHx] = G(xhat_kk_1,params.Ts,y_k);      
    GHs = GHx(posH_row,:);
    
    % project covariance (eq. 12)
    Phat_kk_1 = GFx*Phat_0*GFx' + GFw*params.Q*GFw';    

    % update
    if ~isempty(posH_row)

        % correction term (eq. 13)        
        Ks = (Phat_kk_1*GHs')*(pinv(GHs*Phat_kk_1*GHs' + params.R(posH_row,posH_row)));        

        % correction step (eq. 14-15)
        xnew = xhat_k + Ks*(y_ks-yhat_ks);
        Pnew = Phat_kk_1 - Ks*GHs*Phat_kk_1;
    else
        xnew = xhat_k;
        Pnew = Phat_kk_1;
    end

    %%% TEST %%%    
    xnew(params.pos_bias) = xnew(params.pos_bias);

    % update
    obs.init.X_est(traj).val(:,k) = xnew;
    obs.init.params.Phat(traj).val(k,:,:) = Pnew;

end

%%%%
% Jacobian of df/d(x,w) and dh/dx
function [GFx, GFw, GHx] = G(x,T,y)    

    %%% for the time being the centripetal mode is not included so there is
    %%% no need to add the derivatives of the mappings wrt omega, quat,
    %%% alpha.

    % df/dx
    %      P            V          ACC                B             Q             W             J           ALPHA      B'    
    GFx = [eye(3)       T*eye(3)   0.5*T^2*eye(3)     zeros(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);   ... P
           zeros(3)     eye(3)     T*eye(3)           zeros(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);   ... V
           zeros(3)     zeros(3)   eye(3)             zeros(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);   ... ACC
           zeros(3)     zeros(3)   zeros(3)           1*eye(3)        zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3); ... B
           zeros(4,3)   zeros(4,3) zeros(4,3)         zeros(4,3)    zeros(4)      zeros(4,3)    zeros(4,3)  zeros(4,3) zeros(4,3); ... Q
           zeros(3)     zeros(3)   zeros(3)           zeros(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);   ... OMEGA
           zeros(3)     zeros(3)   zeros(3)           zeros(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);   ... JERK
           zeros(3)     zeros(3)   zeros(3)           zeros(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);   ... ALPHA
           zeros(3)     zeros(3)   zeros(3)           zeros(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);   ... B'
           ];

    % df/dw    
    %      J               ALPHA        B'  
    GFw = [T^3/6*eye(3)    zeros(3)     zeros(3);   ... P
           0.5*T^2*eye(3)  zeros(3)     zeros(3);   ... V
           T*eye(3)        zeros(3)     zeros(3);   ... ACC
           zeros(3)        zeros(3)     T*eye(3);   ... B
           zeros(4,3)      zeros(4,3)   zeros(4,3); ... Q
           zeros(3)        zeros(3)     zeros(3);   ... OMEGA
           zeros(3)        zeros(3)     zeros(3);   ... JERK
           zeros(3)        zeros(3)     zeros(3);   ... ALPHA
           zeros(3)        zeros(3)     zeros(3);   ... B'
           ];

    % dh/dx
    %      P            V          ACC                B             Q             W             J           ALPHA      B'    
    GHx = [eye(3)       zeros(3)   zeros(3)           1*eye(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);    ... P
           zeros(4,3)   zeros(4,3) zeros(4,3)         zeros(4,3)    eye(4)        zeros(4,3)    zeros(4,3)  zeros(4,3) zeros(4,3);  ... Q
           zeros(3)     zeros(3)   eye(3)             0*eye(3)      zeros(3,4)    zeros(3)      zeros(3)    zeros(3)   zeros(3);    ... ACC
           zeros(3)     zeros(3)   zeros(3)           zeros(3)      zeros(3,4)    eye(3)        zeros(3)    zeros(3)   zeros(3);    ... OMEGA
        ];


end