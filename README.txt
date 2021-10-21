# v1.1 - changelog
- main program: simulation_general_v2
- observer class: Lib/obs/obsopt_general_filters
- Filters on measures are implemented (derivative and integral)
- The observer is easy to setup through options (check class constructor)
- Two models have been added as examples. Select the follwing files as 
  @params_init and @measure functions:
	1) simple pendulum: 		Lib/models/model_pendulum AND 
					Lib/models/params_pendulum
	2) runaway electrons shots: 	Lib/models/model_runaway AND
					Lib/models/params_runaway  
- Parameters estimate has been implemented. Examples can be run by selecting 
  the follwing files as @params_init and @measure functions: 
	1) Lib/models/model_pendulum_mass AND Lib/models/params_pendulum_mass
	2) Lib/models/model_runaway_gamma AND Lib/models/params_runaway_gamma
	Remark: remember to correctly set the state dimension

# v1.2 - changelog
- main program: simulation_general_v2
- observer class: Lib/obs/obsopt_general_adaptive
- Adaptive sampling has been implemented. It can be enabled through the        'AdaptiveSampling' boolean flag in the setup.

# how to run the observer in the general example (simulation_general_v2):

MATLAB prompt
>> [params,obs] = simulation_general_v2();

