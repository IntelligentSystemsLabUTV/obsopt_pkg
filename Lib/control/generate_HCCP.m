%% 
function [HCCP, tspan, tspan_pos] = generate_HCCP(Ts,Cnh)

    % time span
    tspan = 0:Ts:60;
    tspan_pos = 1:length(tspan);
    
    % intervals
    tstops = [tspan_pos(1) find(tspan==20) find(tspan==50) tspan_pos(end)];
    tamplitudes = [1*Cnh 0 -0.5*Cnh];
    
    % generate wave
    HCCP = zeros(size(tspan));
    for i=1:length(tamplitudes)
       HCCP(tstops(i):tstops(i+1)) = tamplitudes(i); 
    end

end