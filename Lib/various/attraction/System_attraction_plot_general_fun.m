%% system analysis plot
function System_attraction_plot_general_fun(out,obs,frame,traj,Vfun)

    Niter = length(out);
    frame=min(frame,Niter);


    % transparency
    alphaFace = 0.2;
    alphaEdge = 0.2;

    gotin = 0;
    
    for i=1:Niter

        try
           
            gotin = gotin+1;
                        
            % mesh
            [out{i}.X,out{i}.Y] = meshgrid(out{i}.x_grid(1).val,out{i}.x_grid(2).val);
            out{i}.X = out{i}.X';
            out{i}.Y = out{i}.Y';

            %% plot Hmap numeric
            % plot
            figure(1);
            hold on
%             grid on

            if gotin == frame
                
                % just in case...
                % (1+out{i}.border_hess:end-out{i}.border_hess,1+out{i}.border_hess:end-out{i}.border_hess)
                
                surf(out{i}.X,...
                    out{i}.Y,...
                    out{i}.region,...
                    'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);   
                colormap winter
                freezeColors
                
                bounds_x = [out{i}.x_grid(1).val(1), out{i}.x_grid(1).val(end)];
                bounds_y = [out{i}.x_grid(2).val(1), out{i}.x_grid(2).val(end)];
                vals = [bounds_x(1), bounds_y(1);...
                        bounds_x(1), bounds_y(2);...
                        bounds_x(2), bounds_y(2);...
                        bounds_x(2), bounds_y(1);...
                        bounds_x(1), bounds_y(1)];
                vals = [vals(:,1:2), ones(length(vals),1)];
                plot3(vals(:,1),vals(:,2),vals(:,3),'k--','LineWidth',2);
                
                % plot null gradients
                [row,col] = ind2sub(size(out{i}.V),out{i}.CommonIdx);
                onesVal = ones(length(out{i}.CommonIdx),1);
                plot3(out{i}.x_grid(1).val(row),out{i}.x_grid(2).val(col),onesVal,'+c','LineWidth',2);
               
                
                plot3(out{i}.x0(1),out{i}.x0(2),1,'sk','LineWidth',0.5);
                plot3(out{i}.x0_est_pre(1),out{i}.x0_est_pre(2),1,'ok','LineWidth',0.5);
                plot3(out{i}.x0_est_post(1),out{i}.x0_est_post(2),1,'*k','LineWidth',0.5);
            else
                if traj
                    plot3(out{i}.x0(1),out{i}.x0(2),1,'sg','LineWidth',1);
                    plot3(out{i}.x0_est_pre(1),out{i}.x0_est_pre(2),1,'or','LineWidth',1);
                    plot3(out{i}.x0_est_post(1),out{i}.x0_est_post(2),1,'*b','LineWidth',1);
                end
            end

            % stuff
            xlabel('x_0');
            ylabel('\theta');
            zlabel('Hmap');
            
            %% plot Vfun with points
            if Vfun
                figure(2)
                hold on
                grid on

                if i == 1
                    surf(out{i}.X,...
                        out{i}.Y,...
                        out{i}.V,...
                        'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);   
                    colormap autumn
                    freezeColors
                    plot3(out{i}.x0(1),out{i}.x0(2),0,'sg','LineWidth',2);
                end

                
                Vpost = obs.init.Jstory(i+1);
                plot3(out{i}.x0_est_post(1),out{i}.x0_est_post(2),Vpost,'sb','LineWidth',2);
            end
            
            
        catch
           gotin = gotin -1;
        end
    end
end