%% plot sequence of attraction regions
function sequence_attraction(out,obs)
    Niter = length(out);
    rows = 3;
    cols = ceil(Niter/rows);
    
    figure(4)
    for i=1:Niter
       subplot(rows,cols,i);
       title(['step ',num2str(i)]);
       System_attraction_plot_general_fun_v2(out,obs,i,0,0,0,0,1)
    end
    
end