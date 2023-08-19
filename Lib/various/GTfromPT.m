%%
function [pt, tmp] = GTfromPT(pt,data)

    
    for i=1:size(pt.GTF,1)
        pt.GTFvec(i).val = repmat((pt.GTF(i,:)),[size(data.p,1) 1]);
    end

    fontsize = 15;
    shift = repmat([0 0 -1.3],[size(data.p,1) 1]);
    data.phatfilt = data.phatfilt + shift;
    data.phat = data.phat + shift;

    for i=1:3
        figure(i)
        box on
        hold on
        grid on

        for j=1:numel(pt.GTFvec)
            plot(data.time,pt.GTFvec(j).val(:,i),'k--','LineWidth',0.5);
        end
        
        plot(data.time,data.phat(:,i),'r','LineWidth',0.5);
        plot(data.time,data.phatfilt(:,i),'b','LineWidth',2);
        % labels
        set(gca,'fontsize', fontsize)         
        ylabel(['p_',num2str(i)])

        legend('Pos')   
        xlabel('time [s]')
        xlim('auto');
    end    


    tmp = data;
end