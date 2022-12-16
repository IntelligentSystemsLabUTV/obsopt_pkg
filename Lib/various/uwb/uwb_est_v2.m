function Pr_hat = uwb_est_v2(P_r, P_a, dist_vec, params)
%UWB_EST Summary of this function goes here
%   P_r: vector(2,1) - Rover position
%   P_a: vector(2,n) - Anchor position
%   bias: bool - bias correction
%   dist_vec TODO
%   method: 0 = gradient or 1 = newton, default: Newton
%   epsilon: float - error
    
    % checks

    %%% column vector    
    P_r = reshape(P_r,length(P_r),1);    
    if size(P_r, 1) ~= 2 
        error('ERROR: Rover position size');
        return
    end

    %%% anchor points
    if size(P_a, 1) ~= 2 
        P_a = P_a';
    end
    n = size(P_a,2);
    if size(P_a, 2) ~= params.Nanchor
        error('ERROR: Anchor position size')
        return
    end

    %%% display
    if params.display_uwb
        % display method
        if method == 0
            disp("Choosen method: Gradient")
        else
            disp("Choosen method: Newton")
        end
    end

    %%% bias correction
    if params.bias
        dist_vec = reshape(dist_vec,length(dist_vec),1);        
        if size(dist_vec,1) ~= params.Nanchor
            error('ERROR: dist_vec size')
            return
        end
        for i=1:n
            dist_vec(i) = biasFitting(dist_vec(i));
        end
    end
    
    % calculate exact distance 
    %D_vec(1,:) = sqrt( (P_a(1,:)-P_r(1)).^2 + (P_a(2,:)-P_r(2)).^2);

    % get distance from meas
    D_vec(1,:) = dist_vec;
    
    % amplification factor
    % TODO
    % gamma_i depends on dist_vec(i) variance
    
    N = params.grad_Niter+1;
    
    eta = zeros(2,N);
    % initial conditions (then will be the previous step approximation)
    eta(:,1) = [P_r(1),P_r(2)]*1;
    
    Jn = zeros(N,1);

    % used with gradient method 
    gamma = 0.1;
   
    % test: No optimization
    %N = 1;    

    tic
    for k=2:N
        gradJnum =  params.GJ(eta(1,k-1),eta(2,k-1),P_a,D_vec,params);
        if params.method == 0
            eta(:,k) = eta(:,k-1) - gamma*gradJnum;
        else
            HessianJnum = params.HJ(eta(1,k-1),eta(2,k-1),P_a,D_vec,params);
            eta(:,k) = eta(:,k-1) - pinv(HessianJnum)*gradJnum;
        end
        
        Jn(k) = params.J(eta(1,k-1),eta(2,k-1),P_a,D_vec);
        error = vecnorm(eta(:,k)-P_r,2,1);
        if error < params.epsilon
            break
        end
        %fprintf("Iteration:\t%d\n",k);
    end
    t = toc;
    
    %k = 1;
    Pr_hat = eta(:,k);
    
    if params.display_uwb
        fprintf("Frequency:\t%d Hz\n\n",1/t);
        eta
        error = vecnorm(Pr_hat-P_r)
        figure(j)
        plot(Jn)
        grid on
        legend('Jn')
    end
end

