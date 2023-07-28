%% fcn get distances
function D = get_dist(p,Pa)

    % init
    N = size(Pa,2);
    D = zeros(N,1);

    % compute distances
    for i=1:size(p,2)
        for n = 1:4
            D(4*(i-1)+n) = norm(p(:,i)-Pa(:,n));
        end
    end
end