%% PARAMS_OSCILLATOR_VDP
% file: Params_oscillator_VDP.m
% author: Federico Oliva
% date: 10/01/2022
% description: this function initialises the parameters for a Van der Pol
% oscillator (both for simulation and observation)
% INPUT: none
% OUTPUT:
% Params: structure with all the necessary parameters
function Params = params_oscillator_VDP(varargin)

    % get varargin
    if numel(varargin) > 0
        Ntraj = varargin{1};
    else
        Ntraj = 1;
    end

    % system parameters
    Params.mu = 1;
    Params.A_mu = 0;
    Params.F_mu = 0;
    Params.Phi_mu = 0;
    Params.eps = 1;
    
    % control parameters
    Params.K1 = 0.1;
    Params.K2 = 0.1;

    % init observer buffer (see https://doi.org/10.48550/arXiv.2204.09359)
    Params.N = 10;
    Params.Nts = 10;

    % comment on this
    Params.NtsVal = Params.Nts*ones(1,Params.N);
    
    % number of reference trajectories (under development)
    Params.Ntraj = Ntraj;
    
    % state dimension
    Params.DimState = 3;

    % input dim
    Params.DimInput = 2;

    % output dim
    Params.DimOut = 1;
    Params.DimOutCompare = [1];
    Params.ObservedState = 2;    
    
    % initial condition
    Params.X(1).val(:,1) = [1;1;Params.mu];
    
    % same initial condition for all the trajectories (under development)
    for traj=2:Params.Ntraj
        Params.X(traj).val(:,1) = Params.X(traj-1).val(:,1);
    end

    % initial condtion for the observer 
    for traj=1:Params.Ntraj
        Params.Xhat(traj).val = Params.X(traj).val + randn(Params.DimState,1);
    end
    
    % position in the state vector of the estimated parameters
    Params.EstimatedParams = [3];
    
    % which vars am I optimising
    Params.VarsOpt = [1:3];
    Params.VarsPerturbed = [1:3];
    
    % set the not optimised vars
    tmp = 1:length(Params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(Params.VarsOpt)
        tmp_idx = intersect(tmp_idx,find(tmp~=Params.VarsOpt(i)));
    end
    Params.VarsNonOpt = tmp_idx;
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    Params.VarsPlot = 1:2;
    Params.ParamsPlot = 3;
    Params.VarsMultiTraj = Params.VarsNonOpt;

    % terminal states
    Params.TerminalStates = [1];

    % terminal weights
    Params.OutputWeights = ones(Params.DimOut,1);
    Params.TerminalWeights = ones(Params.DimState,1);

    % barrier fcn
    Params.BoundsPos = [];

end
