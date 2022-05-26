%% TEST_VDP
% file: test_VDP.m
% author: Federico Oliva
% date: 26/05/2022
% description: this file is used to test the good functioning of the lib.
% Please note that the tests assess the status of the main features of the
% tool, not every parameter combination is considered.
% INPUT: no input
% OUTPUT: no output

%%%% init %%%%
% clear workspace
clear

% start time counter
t1 = tic;

% define test structure
test.model = 'VDP';

%% %%%% STANDARD MHE - NOISE - NO PARAMS %%%%

% define test
test.T1.description = 'Standard MHE with noise on Van der Pol oscillator';

% set simulation_general_test
system("sed -i 's/params_init =.*/params_init = @params_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/params_update = @.*/params_update = @params_update_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/model = @.*/model = @model_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/measure = @.*/measure = @measure_general;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/noise_mat =.*/noise_mat = 1*[0,5e-1;0,5e-2];/' Lib/simulation_scripts/test/simulation_general_test.m");

% set params_init
system("sed -i 's/params.dim_state =.*/params.dim_state = 2;/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.X(1).val(:,1) =.*/params.X(1).val(:,1) = [1;1];/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.estimated_params =.*/params.estimated_params = [];/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.opt_vars =.*/params.opt_vars = [1:2];/' Lib/models/params_oscillator_VDP.m");

% set params_update
system("sed -i '/params_out.mu = /s/^/%/' Lib/models/update/params_update_oscillator_VDP.m");

% set filter_define
system("sed -i 's/if.*/if 0/' Lib/measure/filter_define.m");

% run observer
try
    [~,obs] = simulation_general_test;
    test.T1.obs = obs;
    test.T1.result = 'passed';
catch
    test.T1.obs = [];
    test.T1.result = 'failed';
end
test.T1
pause(2);

%% %%%% FILTERED MHE - NOISE - NO PARAMS %%%%
% define test
test.T2.description = 'Filtered MHE with noise on Van der Pol oscillator';

% set simulation_general_test
system("sed -i 's/params_init =.*/params_init = @params_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/params_update = @.*/params_update = @params_update_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/model = @.*/model = @model_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/measure = @.*/measure = @measure_general;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/noise_mat =.*/noise_mat = 1*[0,5e-1;0,5e-2];/' Lib/simulation_scripts/test/simulation_general_test.m");

% set params_init
system("sed -i 's/params.dim_state =.*/params.dim_state = 2;/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.X(1).val(:,1) =.*/params.X(1).val(:,1) = [1;1];/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.estimated_params =.*/params.estimated_params = [];/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.opt_vars =.*/params.opt_vars = [1:2];/' Lib/models/params_oscillator_VDP.m");

% set params_update
system("sed -i '/params_out.mu = /s/^/%/' Lib/models/update/params_update_oscillator_VDP.m");

% set filter_define
system("sed -i 's/if.*/if 1/' Lib/measure/filter_define.m");

% run observer
try
    [~,obs] = simulation_general_test;
    test.T2.obs = obs;
    test.T2.result = 'passed';
catch
    test.T2.obs = [];
    test.T2.result = 'failed';
end
test.T2
pause(2);

%% %%%% STANDARD MHE - NOISE - PARAMS %%%%
% define test
test.T3.description = 'Standard MHE with noise on Van der Pol oscillator - with params estimation';

% set simulation_general_test
system("sed -i 's/params_init =.*/params_init = @params_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/params_update = @.*/params_update = @params_update_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/model = @.*/model = @model_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/measure = @.*/measure = @measure_general;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/noise_mat =.*/noise_mat = 1*[0,5e-1;0,5e-2;0,0];/' Lib/simulation_scripts/test/simulation_general_test.m");

% set params_init
system("sed -i 's/params.dim_state =.*/params.dim_state = 3;/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.X(1).val(:,1) =.*/params.X(1).val(:,1) = [1;1;params.mu];/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.estimated_params =.*/params.estimated_params = [3];/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.opt_vars =.*/params.opt_vars = [1:3];/' Lib/models/params_oscillator_VDP.m");

% set params_update
system("sed -i '18s/\%//g' Lib/models/update/params_update_oscillator_VDP.m");

% set filter_define
system("sed -i 's/if.*/if 0/' Lib/measure/filter_define.m");

% run observer
try
    [~,obs] = simulation_general_test;
    test.T3.obs = obs;
    test.T3.result = 'passed';
catch
    test.T3.obs = [];
    test.T3.result = 'failed';
end
test.T3
pause(2);

%% %%%% FILTERED MHE - NOISE - PARAMS %%%%
% define test
test.T4.description = 'Filtered MHE with noise on Van der Pol oscillator - with params estimation';

% set simulation_general_test
system("sed -i 's/params_init =.*/params_init = @params_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/params_update = @.*/params_update = @params_update_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/model = @.*/model = @model_oscillator_VDP;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/measure = @.*/measure = @measure_general;/' Lib/simulation_scripts/test/simulation_general_test.m");
system("sed -i 's/noise_mat =.*/noise_mat = 1*[0,5e-1;0,5e-2;0,0];/' Lib/simulation_scripts/test/simulation_general_test.m");

% set params_init
system("sed -i 's/params.dim_state =.*/params.dim_state = 3;/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.X(1).val(:,1) =.*/params.X(1).val(:,1) = [1;1;params.mu];/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.estimated_params =.*/params.estimated_params = [3];/' Lib/models/params_oscillator_VDP.m");
system("sed -i 's/params.opt_vars =.*/params.opt_vars = [1:3];/' Lib/models/params_oscillator_VDP.m");

% set params_update
system("sed -i '18s/\%//g' Lib/models/update/params_update_oscillator_VDP.m");

% set filter_define
system("sed -i 's/if.*/if 1/' Lib/measure/filter_define.m");

% run observer
try
    [~,obs] = simulation_general_test;
    test.T4.obs = obs;
    test.T4.result = 'passed';
catch
    test.T4.obs = [];
    test.T4.result = 'failed';
end
test.T4
pause(2);

%% %%%% DRY RUN ON FILTERED MHE - PARAMS %%%%
% define test
test.T5.description = 'Filtered MHE without noise on Van der Pol oscillator - with params estimation';

% run observer
try
    [~,obs] = simulation_general_test_dryrun;
    test.T5.obs = obs;
    test.T5.result = 'passed';
catch
    test.T5.obs = [];
    test.T5.result = 'failed';
end
test.T5

% get final time
test.time = toc(t1);

% tidy up
clear t1 obs