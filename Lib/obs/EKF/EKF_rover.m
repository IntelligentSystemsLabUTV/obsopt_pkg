%% EKF.m
% file: ekf.m
% author: Federico Oliva
% date: 30/01/2023
% description: EKF observer
% INPUT: obs, state, measure
% OUTPUT: obs

function obs = EKF_rover(obs,xhat_kk_1,y_k)

    % get params
    params = obs.init.params;

    % get time instant
    k = obs.init.ActualTimeIndex;
    tspan = obs.setup.time(max(1,k-1):k);

    % get traj
    traj = obs.init.traj;    

    % get control
    uhat_kk_1 = obs.init.input_story_ref(traj).val(:,max(1,k-1));

    % get initial covariance
    sz = size(params.Phat(traj).val);
    Phat_0 = reshape(params.Phat(traj).val(max(1,k-1),:,:),sz(2),sz(3));  

    % prediction step (eq. 11)  NB: check if the correct input is used ins.
    X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.setup.params, obs), tspan, xhat_kk_1,params.odeset); 
    xhat_k = X.y(:,end);

    % predicted measure (and store in obs)
    yhat_k = obs.setup.measure(xhat_k,params,tspan,uhat_kk_1,obs);
    obs.init.Yhat_full_story(traj).val(1,:,k) = yhat_k;

    % init downsampling
    posH_row = [];    
    yhat_ks = [];
    y_ks = [];

    % get downsampling - CAM
    if mod(k,params.UWB_samp_EKF) == 0
        posH_row = [posH_row, [params.pos_dist_out]];
        yhat_ks = [yhat_ks; yhat_k([params.pos_dist_out])];
        y_ks = [y_ks; y_k([params.pos_dist_out])];        
    end

    % get downsampling - IMU
    if mod(k,params.IMU_samp_EKF) == 0
        posH_row = [posH_row, [params.pos_acc_out]];
        yhat_ks = [yhat_ks; yhat_k([params.pos_acc_out])];
        y_ks = [y_ks; y_k([params.pos_acc_out])];
    end

    % get jacobians    
    [GFx, GFw, GHx] = G(xhat_kk_1,params.Ts,y_k,params);      
    GHs = GHx(posH_row,:);
    
    % project covariance (eq. 12)
    Phat_kk_1 = GFx*Phat_0*GFx' + GFw*params.Q*GFw';    

    % update
    if ~isempty(posH_row)

        % correction term (eq. 13)        
        Ks = 1*(Phat_kk_1*GHs')*(pinv(GHs*Phat_kk_1*GHs' + params.R(posH_row,posH_row)));        

        % correction step (eq. 14-15)
        mismatch = (y_ks-yhat_ks);
        correction = Ks*mismatch;

        % test
%         correction([params.pos_p(2:3) params.pos_v(2:3) params.pos_acc(2:3) params.pos_bias(2:3)]) = 0;

        xnew = xhat_k + correction;
        Pnew = Phat_kk_1 - Ks*GHs*Phat_kk_1;
    else
        xnew = xhat_k;
        Pnew = Phat_kk_1;
        correction = zeros(numel(xnew),1);
    end    

    % update
    obs.init.X_est(traj).val(:,k) = xnew;
    obs.init.params.Phat(traj).val(k,:,:) = Pnew;
    obs.init.params.correction_story(:,k) = correction;

end

%%%%
% Jacobian of df/d(x,w) and dh/dx
function [GFx, GFw, GHx] = G(x,T,y,params)    

    %%% for the time being the centripetal mode is not included so there is
    %%% no need to add the derivatives of the mappings wrt omega, quat,
    %%% alpha.
    b = params.bias;
    a = params.proc_acc;
    B = params.bias*params.proc_bias;
    T = params.Ts;

    % df/dx
    %      x1   x2  x3  x4  x5  x6  x7  x8  x9  x10 x11 x12 x13 x14 x15 xothers
    GFx = [...
    %     x dim
           1    T   0   0   0   0   0   0   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x1
           0    1   T   0   0   0   0   0   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x2
           0    0   1   0   0   0   0   0   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x3
           0    0   0   0   0   0   0   0   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x4
           0    0   0   0   1   0   0   0   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x5
    %     y dim
           0    0   0   0   0   1   T   0   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x6
           0    0   0   0   0   0   1   T   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x7
           0    0   0   0   0   0   0   1   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x8
           0    0   0   0   0   0   0   0   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x9
           0    0   0   0   0   0   0   0   0   1   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x10           
    %     z dim
           0    0   0   0   0   0   0   0   0   0   1   T   0   0   0   zeros(1,numel(16:numel(x))); ... x11
           0    0   0   0   0   0   0   0   0   0   0   1   T   0   0   zeros(1,numel(16:numel(x))); ... x12
           0    0   0   0   0   0   0   0   0   0   0   0   1   0   0   zeros(1,numel(16:numel(x))); ... x13
           0    0   0   0   0   0   0   0   0   0   0   0   0   0   0   zeros(1,numel(16:numel(x))); ... x14
           0    0   0   0   0   0   0   0   0   0   0   0   0   0   1   zeros(1,numel(16:numel(x))); ... x15
    %      remaining
           zeros(numel(16:numel(x)),numel(x));                                   ... x16-xend
           ];

    % df/dw
    %      w1   w2  w3  w4  w5  w6
    GFw = [0    0   0   0   0   0;              ... x1
           0    0   0   0   0   0;              ... x2
           a    0   0   0   0   0;              ... x3
           0    0   0   0   0   0;              ... x4
           0    0   0   B   0   0;              ... x5
           0    0   0   0   0   0;              ... x6
           0    0   0   0   0   0;              ... x7
           0    a   0   0   0   0;              ... x8
           0    0   0   0   0   0;              ... x9
           0    0   0   0   B   0;              ... x10
           0    0   0   0   0   0;              ... x11
           0    0   0   0   0   0;              ... x12
           0    0   a   0   0   0;              ... x13
           0    0   0   0   0   0;              ... x14
           0    0   0   0   0   B;              ... x15
           zeros(numel(16:numel(x)),6);          ... x16-xend
           ];

    % dh/dx    
    % adjacency matrix
    for dim=1:params.space_dim
        Pa(dim,:) = x(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end));            
    end
    % get distances
    D = get_dist(x(params.pos_p),Pa);
    GHx = zeros(numel(y),numel(x));
    %      x1               x2  x3  x4  x5  x6              x7  x8  x9  x10 x11             x12 x13 x14 x15  A1x    A1y A1z A2x A2y A2z A3x A3y A3z A4x A4y A4z             
    tmp = [dder(x,1,1,D)    0   0   0   0   dder(x,2,1,D)   0   0   0   0   dder(x,3,1,D)   0   0   0   0    0      0   0   0   0   0   0   0   0   0   0   0;  ... d1
           dder(x,1,2,D)    0   0   0   0   dder(x,2,2,D)   0   0   0   0   dder(x,3,2,D)   0   0   0   0    0      0   0   0   0   0   0   0   0   0   0   0;  ... d2
           dder(x,1,3,D)    0   0   0   0   dder(x,2,3,D)   0   0   0   0   dder(x,3,3,D)   0   0   0   0    0      0   0   0   0   0   0   0   0   0   0   0;  ... d3
           dder(x,1,4,D)    0   0   0   0   dder(x,2,4,D)   0   0   0   0   dder(x,3,4,D)   0   0   0   0    0      0   0   0   0   0   0   0   0   0   0   0;  ... d4
           zeros(6,numel(1:params.pos_anchor(end)));                                                                                                            ... P,V
           0                0   1   0   b   0               0   0   0   0   0               0   0   0   0   0       0   0   0   0   0   0   0   0   0   0   0;  ... a1
           0                0   0   0   0   0               0   1   0   b   0               0   0   0   0   0       0   0   0   0   0   0   0   0   0   0   0;  ... a2
           0                0   0   0   0   0               0   0   0   0   0               0   1   0   b   0       0   0   0   0   0   0   0   0   0   0   0;  ... a3

        ];
    GHx(:,1:params.pos_anchor(end)) = tmp;


end

%%% jacobian of the distance
function dd = dder(x,i,j,D)

    pos_p = [1 6 11];
    pos_anchor = [16:27];

    % get position coordinate
    xp = x(pos_p(i));
    xa = x(pos_anchor((j-1)*3+i));
    dd = (xp-xa)/D(j);

end