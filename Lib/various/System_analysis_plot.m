%% system analysis plot
border_grad = 1;
border_hess = 2;
num_flag = 1 && numeric;
sym_flag = 1 && symbolic;
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
        surf(XVAL,THETA,V_num,'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
        freezeColors
        hold on
    end


    if sym_flag
        % symbolic
        colormap autumn
        surf(XVAL,THETA,V_val,'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
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
        surf(XVAL(1+border_grad:end-border_grad,1+border_grad:end-border_grad),THETA(1+border_grad:end-border_grad,1+border_grad:end-border_grad),GX(1+border_grad:end-border_grad,1+border_grad:end-border_grad),...
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
        surf(XVAL(1+border_grad:end-border_grad,1+border_grad:end-border_grad),THETA(1+border_grad:end-border_grad,1+border_grad:end-border_grad),GTHETA(1+border_grad:end-border_grad,1+border_grad:end-border_grad),...
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
        for i=1:length(state)
            % plot
            figure(i+1);
            grid on

            colormap autumn
            hold on
            tmp_grad = transpose(reshape(V_grad_val(i,:,:),length(x_grid(1).val),length(x_grid(2).val)));
            surf(XVAL(1+border_grad:end-border_grad,1+border_grad:end-border_grad),THETA(1+border_grad:end-border_grad,1+border_grad:end-border_grad),tmp_grad(1+border_grad:end-border_grad,1+border_grad:end-border_grad),...
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
        for i=1:length(state)
            % plot
            figure(5);
            grid on

            colormap spring
            hold on
            tmp_grad = transpose(reshape(V_grad_val(i,:,:),length(x_grid(1).val),length(x_grid(2).val)));
            surf(XVAL(1+border_grad:end-border_grad,1+border_grad:end-border_grad),THETA(1+border_grad:end-border_grad,1+border_grad:end-border_grad),tmp_grad(1+border_grad:end-border_grad,1+border_grad:end-border_grad),...
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
        surf(XVAL(1+border_hess:end-border_hess,1+border_hess:end-border_hess),THETA(1+border_hess:end-border_hess,1+border_hess:end-border_hess),H_map_bool(1+border_hess:end-border_hess,1+border_hess:end-border_hess),...
            'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
        %         surf(XVAL,THETA,H_map);
        freezeColors
    end

    if sym_flag
        colormap autumn
        hold on
        surf(XVAL(1+border_hess:end-border_hess,1+border_hess:end-border_hess),THETA(1+border_hess:end-border_hess,1+border_hess:end-border_hess),H_map_sym_bool(1+border_hess:end-border_hess,1+border_hess:end-border_hess),...
            'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
%         surf(XVAL,THETA,H_map_sym_bool,'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);
    end

    % stuff
    xlabel('x_0');
    ylabel('\theta');
    zlabel('Hmap');
    title('Hmap');
end