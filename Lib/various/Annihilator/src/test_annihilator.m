clearvars
close all
clc

%% Annihilator Test
N = 10;
err = zeros(N, 1);
fprintf('Annihilator Test\n')

% Iterate N times
for i=1:N
    % Generate randomic dimensions
    n = max(randi(11), 2);
    m = max(randi(n-1), 2);
    p = randi(m-1);
    % Generate a stable system with randomic dimensions
    sys_P = rss(n, p, m);

    % Compute a dynamic annihilator for the system
    try
        [sys_An, ~, ~, ~] = annihilator(sys_P);
    catch
       ARARMAX = 1; 
    end

    % Check the behavior of the series between the system and 
    % the annihilator
    try
        sys = series(sys_An, sys_P);
    catch
       ARARMAX = 1; 
    end
    y = step(sys);
    err(i) = max(y, [], 'all');
end

fprintf('Max error %f\n', max(err, [], 'all'))
