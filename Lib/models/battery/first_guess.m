%% main script

function params_out = first_guess(params,params_sim)

    % set time
    t_pos = 1;
    traj = 1;
    
    % set Npoint for cloud
    Npoint = 100;
    
    % define noise
    perc = 1;
    sigma = 0*[0.05 0.2 0.2 0.2];
    
    % define param
    Nparam = 4;
    Npoly = 4;
    CT = 0;
    
    % opt vars
    opt_vars = [7:10,11:14,15:18,19:22,23:26,27:30];
    compare_vars = 3:6;
    update_vars = [ 7 11 15 19 23 27;...
                    8 12 16 20 24 28;...
                    9 13 17 21 25 29;...
                    10 14 18 22 26 30];
    
    % optimset
    myoptioptions = optimset('MaxIter', 300, 'display','off', 'MaxFunEvals',Inf,'TolFun',0,'TolX',0); 
    
    % generate dataset
    x_start = params.X(traj).val(:,1);
    x_out = gen_meas(params_sim,x_start,Npoint,perc,sigma);
    
    % init est
    old_Xest = params.X(traj).val(:,1);
    
    for param = 1:Nparam
        % opt vars
        opt_vars_curr = opt_vars(param + (0:Npoly-1)*Nparam);
        compare_vars_curr = compare_vars(param);
        
        % set the not optimised vars
        tmp = 1:length(params.X(traj).val(:,1));
        tmp_idx = tmp;
        for i=1:length(opt_vars_curr)
            tmp_idx = intersect(tmp_idx,find(tmp~=opt_vars_curr(i)));
        end
        nonopt_vars = tmp_idx;
        
        % evolute for tspan                
        x0 = params.X(traj).val(:,1);
        x0_nonopt = x_out(nonopt_vars,:);
        x0_opt = x0(opt_vars_curr);
    
        % solve the optimisation problem
    %     tmp = fminsearch(@(x)cost_function(obs,x,x0_nonopt,x_out(obs.setup.compare_vars,:),1), x0_opt, myoptioptions); 
        tmp = polyfit(x_out(1,:),x_out(compare_vars_curr,:),Npoly-1);
        
        old_Xest(update_vars(param,1:(Npoly))) = flip(tmp); 
    end        
    
    % obs.init
    params.cloud_Y = x_out(3:6,:);
    params.cloud_X = x_out(1,:);
    
    % update params - init
    update_vars_row = reshape(update_vars(:,1:(Npoly)),1,size(update_vars(:,1:(Npoly)),1)*size(update_vars(:,1:(Npoly)),2));
    
    % if DT
    params.X(traj).val(update_vars_row,1) = old_Xest(update_vars_row);
    
    % update est params
    params = params_update_battery_tushar(params,params.X(traj).val(:,1));       

    params_out = params;
        
end

%%
function x_out = gen_meas(params,x,Npoint,perc,sigma)    

          
    
    NSeg = length(params.input_data.SOC)-1;
    NPointsSeg = floor(Npoint/NSeg);
    
    % initialize state
    x_out = zeros(length(x),NSeg*NPointsSeg);
    
    % create cloud
    for i=1:NSeg
        for j=1:NPointsSeg
            % generate SOC        
            x_out(1,(i-1)*NPointsSeg+j) =  unifrnd(params.input_data.SOC(i),params.input_data.SOC(i+1));            
            
            % generate points (noiseless)
            ref(3) = interp1(params.input_data.SOC(i:i+1), params.input_data.OCV(i:i+1), x_out(1,(i-1)*NPointsSeg+j));
            ref(4) = interp1(params.input_data.SOC(i:i+1), params.input_data.R0(i:i+1), x_out(1,(i-1)*NPointsSeg+j));
            ref(5) = interp1(params.input_data.SOC(i:i+1), params.input_data.R1(i:i+1), x_out(1,(i-1)*NPointsSeg+j));
            ref(6) = interp1(params.input_data.SOC(i:i+1), params.input_data.C1(i:i+1), x_out(1,(i-1)*NPointsSeg+j));
            
            x_out(3,(i-1)*NPointsSeg+j) = ref(3)*(1+sigma(1)*randn);
            x_out(4,(i-1)*NPointsSeg+j) = ref(4)*(1+sigma(2)*randn);
            x_out(5,(i-1)*NPointsSeg+j) = ref(5)*(1+sigma(3)*randn);
            x_out(6,(i-1)*NPointsSeg+j) = ref(6)*(1+sigma(4)*randn);
        
        end
    end  

    [x_out(1,:), I] = sort(x_out(1,:));
    x_out(3,:) = x_out(3,I);
    x_out(4,:) = x_out(4,I);
    x_out(5,:) = x_out(5,I);
    x_out(6,:) = x_out(6,I);
        
        
       
end