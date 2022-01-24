
%% system analysis plot
function System_attraction_plot_general_fun_v2(out,obs,frame,traj,Hmap,contourFlag,Vfun,region)

    Niter = length(out);
    frame=min(frame,Niter);


    % transparency
    alphaFace = 0.4;
    alphaEdge = 0.2;

    gotin = 0;
    
    for i=1:Niter
        
        try
           
            gotin = gotin+1;
            
            if gotin == frame
                MinIdx = out{i}.MinIdx;
                [MinIdxRow,MinIdxCol] = ind2sub(size(out{i}.V),MinIdx);
            end
                        
            % mesh
            [out{i}.X,out{i}.Y] = meshgrid(out{i}.x_grid(1).val,out{i}.x_grid(2).val);
            out{i}.X = out{i}.X';
            out{i}.Y = out{i}.Y';

            %% plot Hmap numeric
            if Hmap
                % plot
                figure(1);
                hold on

                if gotin == frame

                    % just in case...
                    % (1+out{i}.border_hess:end-out{i}.border_hess,1+out{i}.border_hess:end-out{i}.border_hess)

                    surf(out{i}.X,...
                        out{i}.Y,...
                        out{i}.Hmap_bool,...
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

                    plot3(out{i}.x0(1),out{i}.x0(2),1,'sk','LineWidth',0.5);
%                     plot3(out{i}.x0_est_pre(1),out{i}.x0_est_pre(2),1,'ok','LineWidth',0.5);
%                     plot3(out{i}.x0_est_post(1),out{i}.x0_est_post(2),1,'*k','LineWidth',0.5);
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
            end
            
            %% plot Vfun with points
            if contourFlag
                
                Vpost = obs.init.Jstory(i+1);
                
                % contour plot
                figure(2)
                hold on
                grid on

                if gotin == frame
                    x = out{i}.x_grid(1).val;
                    y = out{i}.x_grid(2).val;
                    contour(y,x,out{i}.V)
                    
                    quiver(y,x,out{i}.GV{1},out{i}.GV{2})
                    colorbar
                    plot3(out{i}.x0(2),out{i}.x0(1),0,'sg','LineWidth',2);
                    
%                     plot3(out{i}.x0_est_pre(2),out{i}.x0_est_pre(1),Vpost,'sk','LineWidth',2);
%                     plot3(out{i}.x0_est_post(2),out{i}.x0_est_post(1),Vpost,'sb','LineWidth',2);
                    
                    
%                     surf(out{i}.Y,out{i}.X,out{i}.frontier{3});
                    

                end
                
                if traj
                    plot3(out{i}.x0_est_pre(2),out{i}.x0_est_pre(1),Vpost,'sk','LineWidth',2);
%                     plot3(out{i}.x0_est_post(2),out{i}.x0_est_post(1),Vpost,'sb','LineWidth',2);
                    
                end
                
            end
            
            if Vfun
                
                % surf plot
                figure(3)
                hold on
                grid on

                if gotin == frame
                    surf(out{i}.X,...
                        out{i}.Y,...
                        out{i}.V,...
                        'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge);   
                    colormap summer
                    freezeColors
%                     plot3(out{i}.x0(1),out{i}.x0(2),0,'sg','LineWidth',2);
                end

                
%                 if traj
%                     plot3(out{i}.x0_est_post(1),out{i}.x0_est_post(2),Vpost,'sb','LineWidth',2);
%                 end
            end
            
            if (region) && (gotin == frame)
                figure(4)
                hold on
                
                % region
                surf(out{i}.Y,...
                    out{i}.X,...
                    out{i}.region,...
                    'FaceAlpha',alphaFace,'EdgeAlpha',alphaEdge); 
%                 heatmap(out{i}.region);
                colormap winter
                
                % mins
                plot3(out{i}.x_grid(2).val(MinIdxCol), out{i}.x_grid(1).val(MinIdxRow),out{i}.V(MinIdxRow,MinIdxCol),'.c','LineWidth',2);
                
            elseif (region) && ~(gotin==frame)
                if traj
                    plot3(out{i}.x0_est_pre(2),out{i}.x0_est_pre(1),1,'sk','LineWidth',2);
                end
            end
            
            
        catch
           gotin = gotin -1;
        end
    end
end