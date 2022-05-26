# MHE observer tool
This is a MATLAB tool implementing a Moving Horizon Estimator (MHE). The repository is structured as follows: 
 
- *Lib*: this folder contains all the code. For more information see comments on the code.
- *doc*: this folder contains documentation. 

More specifically, the following info can be useful:

- *Observer_opt_presentation.pdf*: general overview on the theoretical basis for MHEs. More specifically, the presentation covers _standard MHE_, _filtered MHE_, and _adaptive MHE_ (see https://doi.org/10.48550/arXiv.2204.09359).
- *Lib/obs/obsopt.m*: the actual MATLAB class implementing the MHE.
- *Lib/simulation_scripts*: this folder contains the scripts to test and use the MHE.
- Models: the default version of the observer is provided with a Van der Pol oscillator as test example. The user may add its own models following the structure of the files in *Lib/models*

The MHE is designed to work as in the following flow (for more detailed information refer to *simulation_general_v3.m*): 

1) init model and observer
2) for loop
	a) simulate model
	b) get measurements (noise)
	c) run the observer
3) post simulation analysis


# Install and test

First add the library to your MATLAB path and then run the test scripts. On a MATLAB prompt: \
` >> addpath(genpath('Lib'))` \
` >> test_VDP;` \

Now, use *simulation_general.m* as a templte for your code. The file can be run as follows: \
` >> [params,obs] = simulation_general();` \

