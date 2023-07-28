%% unicycle transform 
function x = UnycycleTransformInverse(xhat)

    % theta
    x(1,:) = xhat(1,:);
    
    % x
    x(2,:) = 1./(2*cos(xhat(1,:))).*(xhat(2,:) + (xhat(1,:).*xhat(2,:)-xhat(3,:))/2 );
    
    % y
    x(3,:) = 1./(2*sin(xhat(1,:))).*(xhat(2,:) - (xhat(1,:).*xhat(2,:)-xhat(3,:))/2 );

end