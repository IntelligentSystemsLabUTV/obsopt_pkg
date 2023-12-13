%% fcn get distances
function D = get_dist(p,Pa)

    % init
    N = size(Pa,2);
    D = zeros(N,1);

    % compute distances
    for n = 1:N
        D(n) = norm(p-Pa(:,n));
    end
end