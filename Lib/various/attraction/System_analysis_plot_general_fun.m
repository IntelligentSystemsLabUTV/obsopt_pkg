%% system analysis plot
function System_analysis_plot_general_fun(out)

    if out.StateDim == 2

        % transparency
        alphaFace = 1;
        alphaEdge = 0.2;
        
        % mesh
        [out.X,out.Y] = meshgrid(out.x_grid(1).val,out.x_grid(2).val);
        out.X = out.X';
        out.Y = out.Y';

        %% plot V

        figure(1);
        grid on
        % numeric
        colormap winter
        surf(out.X,out.Y,out.V,'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
        freezeColors
        hold on

        % stuff
        xlabel('x_0');
        ylabel('\theta');
        zlabel('V');
        title('V');

        %% plot gradient 
        % numeric
        %%% gradient on X %%%
        % plot
        figure(2);
        grid on
        colormap winter
        surf(out.X(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
            out.Y(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
            out.GV{1}(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
            'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
        freezeColors

        % stuff
        xlabel('x_0');
        ylabel('\theta');
        zlabel('GX');
        title('GX');

        %%% gradient on THETA %%%
        % plot
        figure(3);
        colormap winter
        surf(out.X(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
            out.Y(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
            out.GV{2}(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
            'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
        freezeColors

        % stuff
        xlabel('x_0');
        ylabel('\theta');
        zlabel('GTHETA');
        title('GTHETA');

        %% plot Hmap numeric
        % plot
        figure(4);
        grid on

        colormap winter
        surf(out.X(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
            out.Y(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
            out.Hmap_bool(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
            'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
        freezeColors

        % stuff
        xlabel('x_0');
        ylabel('\theta');
        zlabel('Hmap');
        title('Hmap');
    end
end