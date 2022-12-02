function Pr_hat = uwb_est(P_r, P_a, bias, dist_vec, method, epsilon, display)
%UWB_EST Summary of this function goes here
%   P_r: vector(2,1) - Rover position
%   P_a: vector(2,n) - Anchor position
%   bias: bool - bias correction
%   dist_vec TODO
%   method: 0 = gradient or 1 = newton, default: Newton
%   epsilon: float - error
    
    % checks
    % column vector
    if size(P_r, 1) == 1
        P_r = P_r';
    end
    if size(P_r, 1) ~= 2 
        disp("ERROR: Rover position size")
        return
        
    end
    if size(P_a, 1) ~= 2 
        P_a = P_a';
    end
    n = size(P_a,2);
    if size(P_a, 1) ~= 2 || size(P_a, 2) ~= n
        disp("ERROR: Anchor position size")
        return
    end
    if display
        % display method
        if method == 0
            disp("Choosen method: Gradient")
        else
            disp("Choosen method: Newton")
        end
    end
    % bias correction
    if bias
        if size(dist_vec,1) == 1
            dist_vec = dist_vec';
        end
        if size(dist_vec,1) ~= n
            disp("ERROR: dist_vec size")
            return
        end
        for i=1:n
            dist_vec(i) = biasFitting(dist_vec(i));
        end
    end
    
    % calculate exact distance 
    %d = zeros(1,n)';
    for i=1:n
        %d(i) = sqrt((P_r(1)-P_a(1,i))^2+(P_r(2)-P_a(2,i))^2);
        eval(['d' num2str(i) ' = sqrt((P_a(1,' num2str(i) ')-P_r(1))^2 +(P_a(2,' num2str(i) ')-P_r(2))^2);']);
    end
    
    % amplification factor
    % TODO
    % gamma_i depends on dist_vec(i) variance
    
    syms x y real
    
    J = 0;
    for i=1:n
        J = J + eval(['(sqrt((P_a(1,' num2str(i) ')-x)^2 +(P_a(2,' num2str(i) ')-y)^2) -d' num2str(i) ')^2;']);
    end
    
    % one shot calculation of gradient and hessian
    gradJ =  gradient(J,[x,y]);
    HessianJ = hessian(J,[x,y]);
    
    % iterations depends on the choosen method
    if method == 0 
        N = 10;
    else
        N = 5;
    end
    
    eta = zeros(2,N);
    % initial conditions (then will be the previous step approximation)
    eta(:,1) = [P_r(1),P_r(2)]*1;
    
    Jn = zeros(N,1);

    % used with gradient method 
    gamma = 0.1;
   
    tic
    for k=2:N
        gradJnum =  eval(subs(gradJ,{x,y},{eta(1,k-1),eta(2,k-1)}));
        if method == 0
            eta(:,k) = eta(:,k-1) - gamma*gradJnum;
        else
            HessianJnum = eval(subs(HessianJ,{x,y},{eta(1,k-1),eta(2,k-1)}));
            eta(:,k) = eta(:,k-1) - pinv(HessianJnum)*gradJnum;
        end
        
        Jn(k) = eval(subs(J,{x,y},{eta(1,k-1),eta(2,k-1)}));
        error = vecnorm(eta(:,k)-P_r,2,1);
        if error < epsilon
            break
        end
        %fprintf("Iteration:\t%d\n",k);
    end
    t = toc;
    
    Pr_hat = eta(:,k);
    
    if display
        fprintf("Frequency:\t%d Hz\n\n",1/t);
        eta
        error = vecnorm(Pr_hat-P_r)
        figure(j)
        plot(Jn)
        grid on
        legend('Jn')
    end
end

