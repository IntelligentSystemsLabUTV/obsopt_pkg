%% control
% file: control.m
% author: Federico Oliva
% date: 20/12/2023
% description: this function implements a general control law 
% INPUT:
%           t: time instant (could be used - see ref)
%           drive: vector containing the values used for the control. See
%           ref for more information
%           Params: structure with all the necessary parameter to the model
%           obs: obsopt instance. It is added in case you need get
%           something from the class storage (could be used)
% OUTPUT:
%           u: control vector
function u = control(t,drive,Params,obs)

    % init input
    u = zeros(Params.DimInput,length(t));

end