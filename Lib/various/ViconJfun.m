%% function to estimate the bias in the vicon positioning
function e = ViconJfun(obs)

    e0 = zeros(2,1);
    params = obs.init.params;
    D_meas = squeeze(obs.init.Y_full_story(1).val(1,params.pos_dist_out,:));
    Xtrue = obs.init.X(1).val;

    opt = optimoptions('fminunc', 'MaxIter', 500, 'display','iter');
    e = fminunc(@(x)Jfun(x,D_meas,Xtrue,params),e0,opt); 

%     opt = optimset('MaxIter', 100,'display','iter');  
%     e = fminsearch(@(x)Jfun(x,D_meas,Xtrue,params),e0,opt); 

end

%% cost function
function J = Jfun(x,DMEAS,Xtrue,params)

    J = 0;
    
    % adjacency matrix
    for dim=1:params.space_dim
        Pa(dim,:) = Xtrue(params.pos_anchor(dim):params.space_dim:params.pos_anchor(end),1);            
    end  

    % all simulation
    for i=params.UWB_samp:params.UWB_samp:params.Niter

        pos = min(max(1,i),params.Niter);

        P_true = Xtrue(params.pos_p,pos);
        P_true = P_true + [x; 0];

        % place the tags - no remove tagPos because i am in the gorund truth: z
        % of the conrol + z of the tag
        Quat_true = Xtrue(params.pos_quat,pos);
        [Y, P, R] = quat2angle(Quat_true','ZYX');
        R = angle2dcm(Y,P,R);
        TagTmp = params.TagPos.*([ones(2,3); 0*ones(1,3)]);
        for j=1:3
            Pt(:,j) = R*TagTmp(:,j) + P_true;
        end
        Pt(3,:) = Pt(3,:) + params.TagPos(3,:);

        % true distances
        D = get_dist(Pt,Pa);
        Dstar = DMEAS(:,pos);

   
        J = J + vecnorm(D-Dstar);
        
    end

    a=1;

end