%% 
function out = Montecarlo_analyse(path,style)   
    
    % TRUE
    params.KX11 = 0.6308;
    params.KX12 = -0.2920;
    params.KX13 = 0.3080;
    params.KX21 = -0.3814;
    params.KX22 = 0.4011;
    params.KX23 = -0.7166;
    params.KX31 = 0.2405;
    params.KX32 = 0.4340;
    params.KX33 = -0.6664;
    
    params.KR11 = 0.0768;
    params.KR12 = -1.3126;
    params.KR13 = -0.1809;
    params.KR21 = 0.4654;
    params.KR22 = 1.5957;
    params.KR23 = 0.7012;
    params.KR31 = -0.4231;
    params.KR32 = 0.3332;
    params.KR33 = 0.6604;
    
    params.KX = [params.KX11, params.KX12, params.KX13; ...
                 params.KX21, params.KX22, params.KX23; ...
                 params.KX31, params.KX32, params.KX33];
    params.KR = [params.KR11, params.KR12, params.KR13; ...
                 params.KR21, params.KR22, params.KR23; ...
                 params.KR31, params.KR32, params.KR33];
             
    list = dir(path);
    list(1:2) = [];
    Nfiles = length(list);  
    
    for i=1:Nfiles
       load([path,list(i).name]);
       out.SNR(i) = norm(obs.init.SNR.val);
       out.KX_err_norm(i) = norm(params.KX-obs.init.params.KX);
       out.KR_err_norm(i) = norm(params.KX-obs.init.params.KX);
       out.avg_time(i) = obs.init.total_time;
    end
    
    figure(1)
    plot(out.SNR,out.KX_err_norm,style);
    set(gca, 'XScale', 'log')
    set(gca, 'YScale', 'log')
    grid on
    box on
    hold on
    
    figure(2)
    plot(out.SNR,out.KR_err_norm,style);
    set(gca, 'XScale', 'log')
    set(gca, 'YScale', 'log')
    grid on
    box on
    hold on
    
end