%% measure function
function u = inject(t,y,yhat,params)


    %% cntrol law - VDP
    u(1) = params.K1*(y(1)-yhat(1));
    u(2) = params.K2*(y(1)-yhat(1));
    
    % input enable
    if ~params.input_enable
        u = zeros(params.dim_input,1);
    end
end