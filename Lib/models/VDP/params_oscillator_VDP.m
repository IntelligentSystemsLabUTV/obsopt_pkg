%% params_oscillator_VDP
% file: params_oscillator_VDP.m
% author: Federico Oliva
% date: 20/12/2023
% description: this function initialises the parameters for a Van der Pol
%              oscillator
% info: see obsopt_manual.pdf for reference
% INPUT: 
%           Ntraj (optional) -> see ref
% OUTPUT: 
%           Params: structure with all the necessary parameters
function Params = params_oscillator_VDP(varargin)

    % get varargin and assign the number of trajectories
    if numel(varargin) > 0
        Ntraj = varargin{1};
    else
        Ntraj = 1;
    end

    % system parameters (see ref)
    Params.mu = 1;  
    Params.eps = 1;
    
    % control parameters (see ref)
    Params.K1 = 0.1;
    Params.K2 = 0.1;

    % init observer buffer (see ref)
    Params.N = 10;
    Params.Nts = 40;

    % here we init a vector of lenggth N with the down-sampling factor Nts.
    % we do this beacuse the down-sampling could change in time 
    % (ref Adaptive MHE) and so it becomes necessary to keep track of the
    % samples distribution in the last measurement buffer
    Params.NtsVal = Params.Nts*ones(1,Params.N);
    
    % number of reference trajectories (see ref)
    Params.Ntraj = Ntraj;
    
    % state dimension
    Params.DimState = 3;

    % input dim
    Params.DimInput = 2;

    % output dim
    Params.DimOut = 2;

    % the standard measurement model is y=Cx, where a subset of the state
    % vector x is taken. This array defines the index of the state to be
    % collected. e.g., [1] for the first element, [1 3] for the first and
    % third element
    Params.ObservedState = [1 2];  
    
    % the MHE is based on the output mismatch (y-yhat). Indeed, one could
    % be interested in considering in the optimization problem not the
    % entire output vector but only a subset of it. This parameter defines
    % the index of the output vector elements to be considered in the
    % mismatch. e.g., if I have a dim=2 output vector and I want to
    % consider only the first element in the optimization i will put
    % DimOutCompare = [1]. If I want to consider only the second element, I
    % will use DimOutCompare = [2]
    Params.DimOutCompare = [1];

    % terminal states variables. If you want to use an arrival cost in the
    % MHE (see ref) you should say on which vars you want it. In this case
    % we are setting an arrival cost on the first state element. It would
    % be [1 2] if we needed it on the first two elements.
    Params.TerminalStates = [1];

    % output weights: this is the W1 matrix in the MHE cost function (ref)
    Params.OutputWeights = ones(Params.DimOut,1);

    % terminal weights: this is the W2 matrix in the MHE cost function (ref)
    Params.TerminalWeights = ones(Params.DimState,1);

    % barrier fcn: in the cost function we could use a barrier function as
    % we did in (BatteryPaper). Here we detail the state vector elements on
    % which we want the barrier. In this case we consider a barrier on the
    % Params.mu parameter (Theta). The bounds can be defined in the obsopt
    % initialization. This could be an option when the LB/UB constraints
    % are not working in "fminsearchcon"
    Params.BoundsPos = [3];
    
    % initial condition of he nominal plant. This should consider the true
    % values of the plant. As explained in ref, if possible obsopt keeps 
    % track of both the nominal and estimated plant. So, it makes sense to
    % store the nominal plant initial condition, if available. 
    Params.X(1).val(:,1) = [1;1;Params.mu];
    
    % same initial condition for all the trajectories (see ref)
    for traj=2:Params.Ntraj
        Params.X(traj).val(:,1) = Params.X(traj-1).val(:,1);
    end

    % initial condtion for the estimated plant. Here we detail for every
    % trajectory the initial estimate that will be provided to the observer
    % We add a random noise just to highlight that the observer does not
    % start from the true condition, but only from an estimate.
    for traj=1:Params.Ntraj
        Params.Xhat(traj).val = Params.X(traj).val + randn(Params.DimState,1);
    end
    
    % position in the state vector of the estimated parameters (Theta in ref)
    Params.EstimatedParams = [3];
    
    % which vars are we optimizing on
    Params.VarsOpt = [1:3];
    
    % set the not optimized vars. This is just a loop automatically getting
    % the rest of the vars from VarsOpt in the state
    tmp = 1:length(Params.X(1).val(:,1));
    tmp_idx = tmp;
    for i=1:length(Params.VarsOpt)
        tmp_idx = intersect(tmp_idx,find(tmp~=Params.VarsOpt(i)));
    end
    Params.VarsNonOpt = tmp_idx;
    
    % plot vars (used to plot the state estimation. When the parameters are
    % too many, consider to use only the true state components)
    Params.VarsPlot = 1:2;      % should be xi in ref
    Params.ParamsPlot = 3;      % should be Theta in ref

end
