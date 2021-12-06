%% 
function out_array = System_analysis_Montecarlo

    
    filter = 0;
    Nw = 5;
    Nts = 10;
    Nsamples = [10;10];
    
    
%     x0 = logspace(-2,-1,5);
%     theta0 = logspace(-2,-1,5);

    x0 = linspace(1,5,10);
    theta0 = linspace(1,5,10);
    offset = 0;
    
    out_array = cell(length(x0),length(theta0),length(offset));
    
    iter = 1;
    Niter = length(x0)*length(theta0)*length(offset);
    
    % simulations
    for a=1:length(x0)
        
        for b=1:length(theta0)
            for c=1:length(offset)
                clc
                disp(['Iteration Number: ', num2str(iter),'/',num2str(Niter)]);
                iter = iter+1;
                
                init = [x0(a); theta0(b)];
                init_offset = offset(c);
                out_array{a,b,c} = System_analysis_fun_general(init,init_offset,filter,Nw,Nts,Nsamples);
            end
        end
    end
end