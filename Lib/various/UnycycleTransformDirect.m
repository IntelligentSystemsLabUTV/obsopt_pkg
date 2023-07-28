%% unicycle transform 
function xhat = UnycycleTransformDirect(x)

    % theta
    xhat(1,:) = x(1,:);
    
    % x
    xhat(2,:) = x(2,:).*cos(x(1,:)) + x(3,:).*sin(x(1,:));
    
    % y
    xhat(3,:) = x(1,:).*(x(2,:).*cos(x(1,:)) + x(3,:).*sin(x(1,:))) - 2*(x(2,:).*cos(x(1,:)) - x(3,:).*sin(x(1,:)));

end