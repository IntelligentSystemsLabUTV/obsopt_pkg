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
    P = params;

    % short state
    Xpos = [P.pos_p, P.pos_v, P.pos_bias, P.pos_acc, P.pos_quat, P.pos_w, P.pos_bias_w];

    % get time instant
    k = obs.init.ActualTimeIndex;
    tspan = obs.setup.time(max(1,k-1):k);

    % get traj
    traj = obs.init.traj;    

    % get control
    uhat_kk_1 = obs.init.input_story_ref(traj).val(:,max(1,k-1));

    % get initial covariance
    sz = size(P.Phat(traj).val);
    if mod(k,P.PhatReset) == 0
        P.Phat(traj).val(max(1,1),:,:) = P.PhatZero;
    end
    Phat_0 = reshape(P.Phat(traj).val(max(1,1),:,:),sz(2),sz(3));  
    Phat_0 = Phat_0(Xpos,Xpos);

    % prediction step (eq. 11)  NB: check if the correct input is used ins.
    try
        X = obs.setup.ode(@(t,x)obs.setup.model(t, x, obs.setup.params, obs), tspan, xhat_kk_1,P.odeset); 
    catch
        warning('EKF: 1 element in time span. Setting integration to initial condition.')
        X.y(:,1) = xhat_kk_1;
    end
    xhat_k = X.y(:,end);

    % predicted measure (and store in obs)
    yhat_k = obs.setup.measure(xhat_k,P,tspan,uhat_kk_1,obs);
    obs.init.Yhat_full_story(traj).val(1,:,k) = yhat_k;

    % init downsampling
    posH_row = [];    
    yhat_ks = [];
    y_ks = [];

    % get downsampling - UWB
    if mod(k,P.UWB_samp) == 0
        posH_row = [posH_row, [P.pos_dist_out]];
        yhat_ks = [yhat_ks; yhat_k([P.pos_dist_out])];
        y_ks = [y_ks; y_k([P.pos_dist_out])];        
    end

    % get downsampling - IMU
    if mod(k,P.IMU_samp) == 0
        posH_row = [posH_row, [P.pos_acc_out]];
        yhat_ks = [yhat_ks; yhat_k([P.pos_acc_out])];
        y_ks = [y_ks; y_k([P.pos_acc_out])];
    end

    % get downsampling - GYRO
    if mod(k,P.Gyro_samp) == 0
        posH_row = [posH_row, [P.pos_w_out]];
        yhat_ks = [yhat_ks; yhat_k([P.pos_w_out])];
        y_ks = [y_ks; y_k([P.pos_w_out])];
    end

    % get jacobians    
    [GFx, GFw, GHx] = G(xhat_kk_1,P.Ts,y_k,P);      
    GHs = GHx(posH_row,:);

    % test sensitivity
    if mod(k,P.UWB_samp) == 0
        obs.init.GHx(k,:,:) = GHx; 
    end

    O = obsv(GFx,GHs);
    obs.init.OBS(k) = rank(O);
        

    % resize
    xhat_kk_1 = xhat_kk_1(Xpos);
    xhat_k = xhat_k(Xpos);
    
    % project covariance (eq. 12)
    Phat_kk_1 = GFx*Phat_0*GFx' + GFw*P.Q*GFw';    

    % update
    if ~isempty(posH_row)

        % correction term (eq. 13)        
        Ks = 1*(Phat_kk_1*GHs')*(pinv(GHs*Phat_kk_1*GHs' + P.R(posH_row,posH_row)));        

        % correction step (eq. 14-15)
        mismatch = (y_ks-yhat_ks);
        correction = Ks*mismatch;

        xnew = xhat_k + 1*correction;
        xnew(P.pos_quat) = quatnormalize(xnew(P.pos_quat)');
        Pnew = Phat_kk_1 - Ks*GHs*Phat_kk_1;
    else
        xnew = xhat_k;
        Pnew = Phat_kk_1;
        correction = zeros(numel(xnew(P.pos_trasl)),1);
    end    

    % update
    obs.init.X_est(traj).val(Xpos,k) = xnew;
    obs.init.params.Phat(traj).val(1,Xpos,Xpos) = Pnew;
    obs.init.params.correction_story(:,k) = correction;

end

%%%%
% Jacobian of df/d(x,w) and dh/dx
function [GFx, GFw, GHx] = G(x,T,y,P)   

    % quaternion
    q = x(P.pos_quat);

    % position
    p = x(P.pos_p);

    % angula velocity
    w = x(P.pos_w);

    % tag position
    delta = P.TagPos;

    % Rotation matrix derivative
    [M1, M2, M3, M4] = M(q);

    %%% df/dx %%%
    GFx = zeros(P.dim_state);
    % dp/dv
    GFx(P.pos_p,P.pos_v) = eye(P.space_dim);
    % dv/da
    GFx(P.pos_v,P.pos_acc) = eye(P.space_dim);
    % dq/dq
    O = OMEGA(w);
    GFx(P.pos_quat,P.pos_quat) = 0.5*O;
    % dq/dw
    [S1, S2, S3] = S();
    GFx(P.pos_quat,P.pos_w) = 0.5*[S1*q S2*q S3*q];

    % to discrete
    Xpos = [P.pos_p, P.pos_v, P.pos_bias, P.pos_acc, P.pos_quat, P.pos_w, P.pos_bias_w];
    Xpospos = [P.pos_p, P.pos_v, P.pos_bias, P.pos_acc];
    Xposquat = [P.pos_quat, P.pos_w, P.pos_bias_w];
    % GFx(Xpos,Xpos) = eye(numel(Xpos)) + GFx(Xpos,Xpos)*T;
    GFx(Xpospos,Xpospos) = eye(numel(Xpospos)) + GFx(Xpospos,Xpospos)*T;
    GFx(Xposquat,Xposquat) = eye(numel(Xposquat)) + GFx(Xposquat,Xposquat)*T;

    %%% df/dw %%%
    Wpos_bias = 1:3;
    Wpos_acc = 4:6;
    Wpos_w = 7:9;
    Wpos = [Wpos_bias Wpos_acc Wpos_w];
    Wpospos = [Wpos_bias Wpos_acc];
    Wposquat = [Wpos_w];
    GFw = zeros(P.dim_state,numel(Wpos));

    % df/dw
    GFw(P.pos_bias,Wpos_bias) = eye(3);
    GFw(P.pos_acc,Wpos_acc) = eye(3);
    GFw(P.pos_w,Wpos_w) = eye(3);

    %%% dh/dx %%%    
    GHx = zeros(P.OutDim,P.dim_state);
    % dyd/dx
    % cycle over the tags and anchors
    for i=1:P.Ntags
        for j=1:P.Nanchor

            % Lij
            Lij = L(x,i,j,P);

            % MU
            [MUij_x, MUij_y, MUij_z] = MU(x,i,j,P);

            % dydij/dp
            GHx(P.pos_dist_out(P.Nanchor*(i-1)+j),P.pos_p) = 1/Lij*[MUij_x, MUij_y, MUij_z];

            % dydij/dq
            GHx(P.pos_dist_out(P.Nanchor*(i-1)+j),P.pos_quat(1)) = 1/Lij*[MUij_x, MUij_y, MUij_z]*M1*delta(:,i);
            GHx(P.pos_dist_out(P.Nanchor*(i-1)+j),P.pos_quat(2)) = 1/Lij*[MUij_x, MUij_y, MUij_z]*M2*delta(:,i);
            GHx(P.pos_dist_out(P.Nanchor*(i-1)+j),P.pos_quat(3)) = 1/Lij*[MUij_x, MUij_y, MUij_z]*M3*delta(:,i);
            GHx(P.pos_dist_out(P.Nanchor*(i-1)+j),P.pos_quat(4)) = 1/Lij*[MUij_x, MUij_y, MUij_z]*M4*delta(:,i);

        end
    end
    % dyw/dw
    GHx(P.pos_w_out,P.pos_w) = eye(3);
    % dya/dba
    GHx(P.pos_acc_out,P.pos_bias) = eye(3);
    % dya/dbacc
    GHx(P.pos_acc_out,P.pos_acc) = eye(3);

    % resize
    GFx = GFx(Xpos,Xpos);
    GFw = GFw(Xpos,:);
    GHx = GHx(:,Xpos);

end

%%% get the Omega matrix
function [S1, S2, S3] = S()

    S1 = [   0    0    0    1;
             0    0   -1    0;
             0    1    0    0;
            -1    0    0    0
         ];

    S2 = [   0    0    1    0;
             0    0    0    1;
            -1    0    0    0;
             0   -1    0    0
         ];

    S3 = [   0   -1    0    0;
             1    0    0    0;
             0    0    0    1;
             0    0   -1    0
         ];
end

%%% get the Omega matrix
function O = OMEGA(W)
    S = [0      -W(3)   +W(2); ...
         +W(3)  0       -W(1); ...
         -W(2)  +W(1)   0];
    O = [+S     W; ...
         -W'    0];
end

%%% get the L norm: i= Tag j = Anchor
function LOUT = L(x,i,j,P)

    [MUx, MUy, MUz] = MU(x,i,j,P);

    % L
    LOUT = norm([MUx, MUy, MUz]);

end

%%% get the Mu vector: i= Tag j = Anchor
function [MUx, MUy, MUz] = MU(x,i,j,P)

    % get position
    p = x(P.pos_p);

    % get quaternion
    q = x(P.pos_quat);
    R = ROT(q);

    % delta
    delta = P.TagPos(:,i);

    % anchor
    A = x(P.pos_anchor(3*(j-1)+[1:3]));

    % MU
    MUx = p(1) + R(1,:)*delta - A(1);
    MUy = p(2) + R(2,:)*delta - A(2);
    MUz = p(3) + R(3,:)*delta - A(3);

end

%%% get the Rotation matrix derivative
function [M1, M2, M3, M4] = M(q)

    M1 = 2*[  +q(1)    +q(4)    -q(3);
              -q(4)    +q(1)    +q(2);
              +q(3)    -q(2)    +q(1);
           ];

    M2 = 2*[  +q(2)    +q(3)    +q(4);
              +q(3)    -q(2)    +q(1);
              +q(4)    -q(1)    -q(2);
           ];

    M3 = 2*[  -q(3)    +q(2)    -q(1);
              +q(2)    +q(3)    +q(4);
              +q(1)    +q(4)    -q(3);
           ];

    M4 = 2*[  -q(4)    +q(1)    +q(2);
              -q(1)    -q(4)    +q(3);
              +q(2)    +q(3)    +q(4);
           ];

end

%%% get the Rotation matrix
function R = ROT(q)

    % column 1
    R(1,1) = q(2)^2 - q(3)^2 - q(4)^2 + q(1)^2;
    R(2,1) = 2*(q(2)*q(3) - q(4)*q(1));
    R(3,1) = 2*(q(2)*q(4) + q(3)*q(1));

    % column 2
    R(1,2) = 2*(q(2)*q(3) + q(4)*q(1));
    R(2,2) = -q(2)^2 + q(3)^2 - q(4)^2 + q(1)^2;
    R(3,2) = 2*(q(3)*q(4) - q(2)*q(1));

    % column 3
    R(1,3) = 2*(q(2)*q(4) - q(3)*q(1));
    R(2,3) = 2*(q(3)*q(4) + q(2)*q(1));
    R(3,3) = -q(2)^2 - q(3)^2 + q(4)^2 + q(1)^2;

end