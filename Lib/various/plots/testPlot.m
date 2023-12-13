%%
for image=1:4
    start = [8 12 16 20];
    figure(image)
    hold on
    for i=1:3
        plot(obs_s{2}.setup.time,obs_s{2}.init.X_est(1).val(start+(i-1),:),'LineWidth',2)
    end
end