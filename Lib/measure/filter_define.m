%% FILTER DEFINE
% file: filter_define.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function defines the filters applied to the output 
% measurements. Their usage is strictly related to filtered_MHE (see 
% https://doi.org/10.48550/arXiv.2204.09359). Simply use the if 1
% enable or disable the filtering actions.
% INPUT: 
% Ts: sampling time
% Nts: down-sampling (see https://doi.org/10.48550/arXiv.2204.09359)
% OUTPUT:
% filter: structure with the filter transfer function and ss realization
% filterScale: array weighting the filters in the cost function
function [filter, filterScale, reference] = filter_define(Ts,Nts)

    % init filterScale and filter
    i = 0;
    filterScale(i+1)= 1;
    filter = [];
    
    %%% derivative filter %%%
    if 0
    i = i+1;   
    eps1 = 1e-1;
    G = tf([1 0],[eps1 1]);
    SS = ss(G);
    D = c2d(SS,Ts);
    filter(i).TF = D;
    filter(i).A = D.A;
    filter(i).B = D.B;
    filter(i).C = D.C;
    filter(i).D = D.D;
    filter(i).G = G;
    filter(i).dim = size(D.B,1);
    filterScale(i+1)= 1;
    end

    %%%% integral filter %%%%
    if 0
    i = i+1;    
    eps2 = 1e2;
    G = tf(1,[eps2 1]);
    SS = ss(G);
    D = c2d(SS,Ts);
    filter(i).TF = D;
    filter(i).A = D.A;
    filter(i).B = D.B;
    filter(i).C = D.C;
    filter(i).D = D.D;
    filter(i).G = G;
    filter(i).dim = size(D.B,1);
    filterScale(i+1)= 1;
    end
    
    %%%% reference filter %%%%
    % (under development)
    eps = 2e0;
    G = tf(eps,[1 eps]);
    SS = ss(G);
    D = c2d(SS,Ts);
    reference.TF = D;
    reference.A = D.A;
    reference.B = D.B;
    reference.C = D.C;
    reference.D = D.D;
    reference.G = G;
    reference.dim = size(D.B,1);
    
end