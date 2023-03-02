%%
clear ex ey ez
start=3000;
for i=1:params.Ntraj
    ex(i,:) = obs.init.X(i).val(1,:)-obs.init.X_est(i).val(1,:);
    ey(i,:) = obs.init.X(i).val(6,:)-obs.init.X_est(i).val(6,:);
    ez(i,:) = obs.init.X(i).val(11,:)-obs.init.X_est(i).val(11,:);
    sigmax((i)) = std(ex(i,start:end));
    sigmay((i)) = std(ey(i,start:end));
    sigmaz((i)) = std(ez(i,start:end));
    meanx((i)) = mean(ex(i,start:end));
    meany((i)) = mean(ey(i,start:end));
    meanz((i)) = mean(ez(i,start:end));
end