%% system analysis plot
function System_analysis_plot_fun(out)

    num_flag = 1 && out.numeric;
    sym_flag = 1 && out.symbolic;
    plot_flag = num_flag || sym_flag;
    alphaFace = 1;
    alphaEdge = 0.2;

    %% plot V
    if plot_flag
        figure(1);
        grid on

        if num_flag
            % numeric
            colormap winter
            surf(out.XVAL,out.THETA,out.V_num,'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
            freezeColors
            hold on
        end


        if sym_flag
            % symbolic
            colormap autumn
            surf(out.XVAL,out.THETA,out.V_sym,'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
        end

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

        if num_flag
            colormap winter
            surf(out.XVAL(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                out.THETA(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                out.V_num_grad(1).val(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
            freezeColors

            % stuff
            xlabel('x_0');
            ylabel('\theta');
            zlabel('GX');
            title('GX');
        end

        %%% gradient on THETA %%%
        % plot
        figure(3);
        grid on

        if num_flag
            colormap winter
            surf(out.XVAL(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                out.THETA(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                out.V_num_grad(2).val(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
            freezeColors

            % stuff
            xlabel('x_0');
            ylabel('\theta');
            zlabel('GTHETA');
            title('GTHETA');
        end

        % symbolic
        if sym_flag
            for i=1:length(out.state)
                % plot
                figure(i+1);
                grid on

                colormap autumn
                hold on
                tmp_grad = transpose(reshape(out.V_sym_grad(i).val,length(out.x_grid(1).val),length(out.x_grid(2).val)));
                surf(out.XVAL(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                    out.THETA(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                    tmp_grad(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                    'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);

                % stuff
                xlabel('x_0');
                ylabel('\theta');
                zlabel('grad');
                title(['GRAD',num2str(i)]);
            end
        end

        % same figure both gradients
        if sym_flag
            for i=1:length(out.state)
                % plot
                figure(5);
                grid on

                colormap spring
                hold on
                tmp_grad = transpose(reshape(out.V_sym_grad(i).val,length(out.x_grid(1).val),length(out.x_grid(2).val)));
                surf(out.XVAL(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                    out.THETA(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                    tmp_grad(1+out.border_grad:end-out.border_grad,1+out.border_grad:end-out.border_grad),...
                    'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);

                % stuff
                xlabel('x_0');
                ylabel('\theta');
                zlabel('grad');
                title(['GRAD',num2str(i)]);
            end
        end

        %% plot Hmap numeric
        % plot
        figure(4);
        grid on

        if num_flag
            colormap winter
            surf(out.XVAL(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
                out.THETA(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
                out.Hmap_bool_num(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
                'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
            freezeColors
        end

        if sym_flag
            colormap autumn
            hold on
            surf(out.XVAL(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
                out.THETA(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
                out.Hmap_bool_sym(1+out.border_hess:end-out.border_hess,1+out.border_hess:end-out.border_hess),...
                'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
        end

        % stuff
        xlabel('x_0');
        ylabel('\theta');
        zlabel('Hmap');
        title('Hmap');
    end
end