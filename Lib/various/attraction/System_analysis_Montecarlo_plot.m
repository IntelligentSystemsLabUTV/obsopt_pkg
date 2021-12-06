%%
function out = System_analysis_Montecarlo_plot(out_array,sym)
    
    for i=1:size(out_array,1)
       for j=1:size(out_array,2)
           for k=1:size(out_array,3)
               
               if sym
                   out.attraction(i,j,k) = out_array{i,j,k}.attraction_sym;
                   out.PE(i,j,k) = out_array{i,j,k}.PE_sym;
               else
                   out.attraction(i,j,k) = out_array{i,j,k}.attraction_num;
                   out.PE(i,j,k) = out_array{i,j,k}.PE_num;
               end
               out.x0(i) = out_array{i,j,k}.x0(1);
               out.theta0(j) = out_array{i,j,k}.x0(2);
               out.offset(k) = out_array{i,j,k}.offset;
           end
       end
    end
    
    xvar = out.x0;
    yvar = out.theta0;
    [X,Y] = meshgrid(xvar,yvar);
    out.attraction = transpose(reshape(out.attraction(:,:,1),length(xvar),length(yvar)));
    out.PE = transpose(reshape(out.PE(:,:,1),length(xvar),length(yvar)));
    [GX, GY] = gradient(out.attraction);
    out.attraction_grad(1).val = GX;
    out.attraction_grad(2).val = GY;
    [GX, GY] = gradient(out.PE);
    out.PE_grad(1).val = GX;
    out.PE_grad(2).val = GY;
    
    figure(1)
    grid on
    surf(X,Y,out.attraction);
    % stuff
    xlabel('X');
    ylabel('Y');
    zlabel('att');
    title('ATTRACTION');
    
    figure(2)
    grid on
    hold on
    colormap winter
    surf(X,Y,out.attraction_grad(1).val);
    freezeColors
    colormap spring
    surf(X,Y,out.attraction_grad(2).val);
    freezeColors
    % stuff
    xlabel('X');
    ylabel('Y');
    zlabel('grad att');
    title('GRADIENT ATTRACTION');
    legend('GX','GY')
    
    figure(3)
    grid on
    surf(X,Y,out.PE);
    % stuff
    xlabel('X');
    ylabel('Y');
    zlabel('PE');
    title('EXCITATION');
    
    figure(4)
    grid on
    hold on
    colormap winter
    surf(X,Y,out.PE_grad(1).val);
    freezeColors
    colormap spring
    surf(X,Y,out.PE_grad(2).val);
    freezeColors
    % stuff
    xlabel('X');
    ylabel('Y');
    zlabel('grad PE');
    title('GRADIENT EXCITATION');
    legend('GX','GY')

end