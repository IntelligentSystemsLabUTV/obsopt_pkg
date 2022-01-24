%% script for the theoretical test
function out = System_analysis_fun_general_v3(x0,offset,filter,Nw,Nts,Nsamples,boundperc,params,obs)

    % flags 
    out.numeric = 1;
    out.symbolic = 0;
    
    % set initial and final time instant
    % these could be different from the optimisation ones
    % params won't be an output so none of these modifications will
    % propagate to the next optimisation step
    t0 = 0;
    tend = (Nts*Nw+offset)*params.Ts;
    params.time = t0:params.Ts:tend;
    params.Niter = length(params.time);
    obs.setup.time = params.time;
    obs.setup.Niter = params.Niter;
    
    % no optimisation, just model propagation
    obs.setup.optimise = 0;
    
    % this is the TRUE state at this time instant, the pivot of the V fun
    params.X = x0;
    
    % hessian threshold
    thresh_eig = 0;

    % define evaluation grid
%     x_range = [boundperc(:,1).*params.X, params.X, boundperc(:,2).*params.X];
    x_range = boundperc;
    for i=1:params.StateDim
        left = linspace(x_range(i,1),x_range(i,2),Nsamples(i));
        right = linspace(x_range(i,2),x_range(i,3),Nsamples(i));
        x_grid(i).val = [left, right(2:end)];
        Tgran(i).val = diff(x_grid(i).val);
    end
    
    tmp_str = '[';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'X(',num2str(i),').mesh,'];
    end
    tmp_str = [tmp_str(1:end-1),'] = meshgrid('];
    for i=1:params.StateDim
        tmp_str = [tmp_str,'x_grid(',num2str(i),').val,'];
    end
    tmp_str = [tmp_str(1:end-1),');'];
    eval(tmp_str);
    
    %%% output
    out.X = X;
    out.x_grid = x_grid;
    out.x_range = x_range;
    out.StateDim = params.StateDim;
    out.Tgran = Tgran;
    out.offset = offset;
    out.Nw = Nw;
    out.Nts = Nts;
    out.Ts = obs.setup.Ts;
    out.filter = filter;
    out.x0 = x0;
    out.thresh_eig = thresh_eig;
    out.border_grad = 1;
    out.border_hess = 2;


    %% COST FUNCTION %%
    % compute cost function
    % init cost function
    tmp_str = 'V_num = zeros(';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'length(x_grid(',num2str(i),').val),'];
    end
    tmp_str = [tmp_str(1:end-1), ');'];
    eval(tmp_str);
    
    % for loop
    % generate StateDim nested for loops with index IT$ with $=1..StateDim
    % inside the loop call the observer_internal fcn --> propagate and
    % compute V. 
    tmp_str = '';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'for IT',num2str(i),'=1:length(x_grid(',num2str(i),').val)',newline];
    end
    tmp_str = [tmp_str, newline, 'IT = ['];
    for i=1:params.StateDim
       tmp_str = [tmp_str,'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1),'];'];
    tmp_str = [tmp_str, newline, '[obs_out, V_num, out] = observer_internal_fun(x_grid,params,obs,out,IT,V_num);', newline, newline];
    for i=1:params.StateDim
       tmp_str = [tmp_str,'end', newline]; 
    end
    eval(tmp_str);

    %% gradient analysis %%
    tmp_str = '[';
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'GV{',num2str(i),'},']; 
    end
    tmp_str = [tmp_str(1:end-1), '] = gradient(V_num,'];
    for i=1:params.StateDim
        tmp_str = [tmp_str, 'Tgran(',num2str(i),').val(1),'];
    end
    tmp_str = [tmp_str(1:end-1), ');'];
    eval(tmp_str); 
    
    % region of attraction
    for i=1:params.StateDim
        GV_sign{i} = sign(GV{i});
    end
    
    % frontiers
    [frontier{1},~] = gradient(GV_sign{1});
    frontier{1} = frontier{1}~=0;
    [~,frontier{2}] = gradient(GV_sign{2});
    frontier{2} = frontier{2}~=0;
    frontier{3} = double(frontier{1} | frontier{2});
    zeroIdx = find(frontier{3} == 0);
    frontier{3}(zeroIdx) = NaN;
    
    out.frontier = frontier;
    out.GV_sign = GV_sign;
    

    %% hessian analysis %%
    % compute the Hessian in all the NsamplesXNsamples grid points. It is a
    % StateDimXStateDim matrix
    for i=1:params.StateDim
        tmp_str = '[';
        for j=1:params.StateDim
            tmp_str = [tmp_str, 'HV{',num2str(i),',',num2str(j),'},']; 
        end
        tmp_str = [tmp_str(1:end-1), '] = gradient(GV{',num2str(i),'},'];
        for j=1:params.StateDim
            tmp_str = [tmp_str, 'Tgran(',num2str(j),').val(1),'];
        end
        tmp_str = [tmp_str(1:end-1), ');'];
        eval(tmp_str); 
    end

    % region of attraction
    H_map = zeros(size(V_num));
    H_map_bool = zeros(size(V_num));
    
    % for loop
    % here we compute the region of attraction depending on the positive
    % definition of the Hessian in each grid point. Cycle over all the grid
    % (IT), then compute the eigenvalues of the Hessian StateDimXStateDim
    % matrix, then create a boolean map where this is positive definite
    tmp_str = '';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'for IT',num2str(i),'=1:length(x_grid(',num2str(i),').val)',newline];
    end

    tmp_str = [tmp_str, 'for i=1:params.StateDim', newline, 'for j=1:params.StateDim', newline];
    tmp_str = [tmp_str, newline, 'Hpoint(i,j) = HV{i,j}('];
    
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    
    tmp_str = [tmp_str(1:end-1), ');',newline, newline];  
    tmp_str = [tmp_str, 'end', newline, 'end', newline]; 
    
    % Det and Trace
    tmp_str = [tmp_str, 'H_det('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = det(Hpoint);', newline];
    tmp_str = [tmp_str, 'H_trace('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = trace(Hpoint);', newline];
    
    % eigs
    tmp_str = [tmp_str, newline, 'tmp_eig=eig(Hpoint);',newline];
    tmp_str = [tmp_str, 'IsPositiveDefinite = ~(any(tmp_eig<thresh_eig));',newline,newline];
    tmp_str = [tmp_str, 'if IsPositiveDefinite', newline];
    % Hmap
    tmp_str = [tmp_str, 'H_map('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = norm(tmp_eig);', newline];
    
    % Hmap bool
    tmp_str = [tmp_str, 'H_map_bool('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = 1;', newline];
    tmp_str = [tmp_str, 'else', newline];
    % Hmap
    tmp_str = [tmp_str, 'H_map('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = -norm(tmp_eig);', newline];
    
    % Hmap bool
    tmp_str = [tmp_str, 'H_map_bool('];
    for i=1:params.StateDim
       tmp_str = [tmp_str, 'IT',num2str(i),',']; 
    end
    tmp_str = [tmp_str(1:end-1), ') = NaN;', newline];
    tmp_str = [tmp_str, 'end', newline,newline];
    
    for i=1:params.StateDim
       tmp_str = [tmp_str,'end', newline]; 
    end
    eval(tmp_str);
    
    H_map_bool_withlims = H_map_bool;
    H_map_bool_withlims(find(isnan(H_map_bool)))= 10;
    H_map_bool_withlims(find(H_map_bool_withlims == 1))= NaN;
    H_map_bool_withlims(find(H_map_bool_withlims == 10))= 1;
    
    
    %% get null gradients
    thresh_grad = 1e-2;
    thresh_val = 1e-1;
    
    Gmap_bool = cell(params.StateDim,1); 
    for i=1:params.StateDim

        % init
        tmp_GV = GV{i};
        Gmap_bool{i}.map = zeros(size(V_num));

        % v max
        minGV = min(min(abs(tmp_GV)));
        tmp_thresh = thresh_grad;

        % find index
        diffGV = abs(abs(tmp_GV) - minGV);
        idx = find(diffGV < tmp_thresh);
        Gmap_bool{i}.map(idx) = 1;
    end
    
    %% get function min and max vals
    [VmaxVal, VmaxIdx] = max(max(V_num));
    [VminVal, VminIdx] = min(min(V_num));
        
    %% find local minima
    tmp_str = 'MinIdx = find(';
    for i=1:params.StateDim
        tmp_str = [tmp_str,'(Gmap_bool{',num2str(i),'}.map == 1) & '];
    end
    tmp_str = [tmp_str(1:end-3), '& (H_map_bool == 1));'];
    eval(tmp_str);
    
    % get a visualisation
    MinTestMatrix = zeros(size(V_num));
    MinTestMatrix(MinIdx) = 1;
    out.MinIdx = MinIdx;
    
    %% region of attraction
    Nmin = length(MinIdx);
    region = cell(Nmin,1);
    for i = 1:Nmin
        
        % init
        [regionIdx1_init, regionIdx2_init] = ind2sub(size(V_num),MinIdx(i));
        regionIdx1 = regionIdx1_init;
        regionIdx2 = regionIdx2_init;
        
        stopflagX = 0;
        stopflagY = 0;
        
        rowmax = size(V_num,1);
        colmax = size(V_num,2);
        
        %%% x axis %%%
        while ~stopflagX
            
            % comparison val
            V0 = V_num(regionIdx1, regionIdx2_init);
                       
            % increase x
            tmpIdx = [max(1,regionIdx1(1) -1), min(colmax,regionIdx1(end) + 1)];
            cond = (V_num(tmpIdx,regionIdx2_init) > V0);
            
            if any(cond)
                if cond(1)
                    regionIdx1(1) = tmpIdx(1);
                end
                if cond(2)
                    regionIdx1(2) = tmpIdx(2);
                end
            else
                stopflagX = 1;
                regionIdx1 = regionIdx1 + [1, -1];
            end
              
        end
        
        %%% y axis %%%  
        while ~stopflagY
            
            % comparison val
            V0 = V_num(regionIdx1_init, regionIdx2);

            % increase y
            tmpIdx = [max(1,regionIdx2(1) -1), min(rowmax,regionIdx2(end) + 1)];
            cond = (V_num(regionIdx1_init,tmpIdx) > V0);
            
            if any(cond )
                if cond(1)
                    regionIdx2(1) = tmpIdx(1);
                end
                if cond(2)
                    regionIdx2(2) = tmpIdx(2);
                end
            else
                stopflagY = 1;
                regionIdx2 = regionIdx2 + [1, -1];
            end
        end
        
        region{i} = zeros(size(V_num));
        region{i}(regionIdx1(1):regionIdx1(end),regionIdx2(1):regionIdx2(end)) = 1;
        
    end
    
    % sum all regions
    totregion = zeros(size(V_num));
    for i=1:Nmin
        totregion = totregion + region{i};
    end
    idx = find(totregion == max(max(totregion)));
    totregion = NaN(size(V_num));
    totregion(idx) = 1;

    %%% output
    out.V = V_num;
    out.GV = GV;
    out.HV = HV;
    out.Hmap = H_map;
    out.Hmap_bool = H_map_bool;
    out.Hmap_bool_withlims = H_map_bool_withlims;
    out.H_det = H_det;
    out.H_trace = H_trace;
    out.region = totregion;

    % compute the area of attraction compared to the whole grid area
    H_map_restricted = H_map(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess);
    H_map_bool_restricted = H_map_bool(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess);
    out.attraction_num = length(find(H_map_bool_restricted==1))/length(find(H_map_bool_restricted));
    
    %% compute f on a grid
    
    
end

