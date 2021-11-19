%MDL_TWOLINK Create model of a 2-link mechanism
%
% MDL_TWOLINK is a script that creates the workspace variable twolink which
% describes the kinematic and dynamic characteristics of a simple planar
% 2-link mechanism moving in the xz-plane, it experiences gravity loading.
%
% Also defines the vector:
%   qz   corresponds to the zero joint angle configuration.
%
% Notes::
% - SI units are used.
% - It is a planar mechanism operating in the vertical plane and is 
%   therefore affected by gravity (unlike mdl_planar2 in the horizontal
%   plane).
% - Assume unit length links with all mass (unity) concentrated at the joints.
%
% References::
%  - Based on Fig 3-6 (p73) of Spong and Vidyasagar (1st edition).  
%
% See also mdl_twolink_sym, mdl_planar2, SerialLink.

% MODEL: generic, planar, dynamics, 2DOF, standard_DH



% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
function twolink = mdl_double_pendulum(params,obs)

    twolink = SerialLink([
        Revolute('d', 0, 'a', params.Lt1, 'alpha', 0)
        Revolute('d', 0, 'a', params.Lt2, 'alpha', 0)
        ], ...
        'name', 'two link');

    twolink.base = trotx(pi/2);
%     twolink.plot(obs.init.X(1:2,:)');
    twolink.plot([-pi/2,-pi/2]);
    
end

