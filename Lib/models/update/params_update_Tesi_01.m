%% function 
function params = params_update_Tesi_01(params,x)
    
    % control
    for i=1:3
       for j=1:3
          eval(['params.KX',num2str(i),num2str(j),' = x(params.dim_state+3*(i-1)+j);']); 
          eval(['params.KR',num2str(i),num2str(j),' = x(params.dim_state+6*(i-1)+2*j);']);
       end
    end
    
    params.KX = [params.KX11, params.KX12, params.KX13; ...
                 params.KX21, params.KX22, params.KX23; ...
                 params.KX31, params.KX32, params.KX33];
    params.KR = [params.KR11, params.KR12, params.KR13; ...
                 params.KR21, params.KR22, params.KR23; ...
                 params.KR31, params.KR32, params.KR33];
end