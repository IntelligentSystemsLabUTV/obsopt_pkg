%% Montecarl simulations
function Montecarlo(folder,N)

    % init 
    noise = logspace(-3,-1,10);
    Nnoise = length(noise);
    seed = 0;
    
    % range among noise
    for n=1:Nnoise
        for r=1:N
           clc
           disp(['Noise: ',num2str(noise(n)),' Run: ',num2str(r)]);
           [~,obs] = simulation_noise_v1([0 noise(n)], seed); 
           
           % true
           KX = [obs.setup.params.KX11, obs.setup.params.KX12, obs.setup.params.KX13; ...
                 obs.setup.params.KX21, obs.setup.params.KX22, obs.setup.params.KX23; ...
                 obs.setup.params.KX31, obs.setup.params.KX32, obs.setup.params.KX33];
           KR = [obs.setup.params.KR11, obs.setup.params.KR12, obs.setup.params.KR13; ...
                 obs.setup.params.KR21, obs.setup.params.KR22, obs.setup.params.KR23; ...
                 obs.setup.params.KR31, obs.setup.params.KR32, obs.setup.params.KR33];
           
           seed = seed + 1;
           save([folder,'/N',num2str(noise(n)),'_R',num2str(r),'.mat']);
        end
    end
    
    load handel
    sound(y,Fs)
end