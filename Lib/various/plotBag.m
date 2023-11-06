%% plot data
function data = plotBag(out,plotF,GTF,tstop,Ts,rescaleFlag)

    % init
    fig_count = 0;
    fontsize = 15;
    alignflag = 1;
    rescaleFlagVicon = rescaleFlag(1);
    rescaleFlagUWB = rescaleFlag(2);
    rescaleFlagEKF = rescaleFlag(3);
    rescaleFlagPJUMP = rescaleFlag(4);
    rescaleFlagIMU = rescaleFlag(5);
    % close all
    set(0,'DefaultFigureWindowStyle','docked');

    % define time of gorund truth and estimation    
    if GTF

        a(1,1) = double(out.viconData{1}.Header.Stamp.Sec);
        a(2,1) = double(out.EKFData{1}.Header.Stamp.Sec);
        a(3,1) = double(out.UWBData{1}.Header.Stamp.Sec);
        a(4,1) = double(out.IMUData{1}.Header.Stamp.Sec);

        a(1,2) = double(out.viconData{end}.Header.Stamp.Sec);
        a(2,2) = double(out.EKFData{end}.Header.Stamp.Sec);
        a(3,2) = double(out.UWBData{end}.Header.Stamp.Sec);
        a(4,2) = double(out.IMUData{end}.Header.Stamp.Sec);

            
    else
       
        a(1,1) = double(out.EKFData{1}.Header.Stamp.Sec);
        a(2,1) = double(out.UWBData{1}.Header.Stamp.Sec);
        a(3,1) = double(out.IMUData{1}.Header.Stamp.Sec);

        a(1,2) = double(out.EKFData{end}.Header.Stamp.Sec);
        a(2,2) = double(out.UWBData{end}.Header.Stamp.Sec);
        a(3,2) = double(out.IMUData{end}.Header.Stamp.Sec);
 
    end
    [T0_GT,i] = min(a(:,1));
    [TF_GT,j] =  max(a(:,2)-a(:,1));   
    time = T0_GT:Ts:(T0_GT + TF_GT);
    time = time - T0_GT;
    time_full = time;

    % set the end time if provided (shorter bag)
    if isempty(tstop)
        val = time(end);
    else
        val = tstop;
    end

    % find the closest instant in the actual time array
    [~, pos] = min(abs(time - val));  

    % shrink the time if possible
    time = time(1:pos);   

    % set the time array
    data.time = time;

    % pos for errors
    startpos = floor(numel(time)/2);
    endpos = numel(time)-1;

    %% position vicon

    % define ground truth
    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Position.X),out.viconData);
    else        
        x = zeros(numel(time),1);
    end    
    data.p(:,1) = x;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Position.Y),out.viconData);
    else        
        x = zeros(numel(time),1);
    end
    data.p(:,2) = x;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Position.Z),out.viconData);
    else        
        x = zeros(numel(time),1);
    end
    data.p(:,3) = x;

    % starting time for Vicon 
    T0_Vicon = double(out.viconData{1}.Header.Stamp.Sec);   % get T0 for Vicon
    TsVicon = Ts; %mean(diff(cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),out.viconData,'UniformOutput',false))));
    dT = T0_GT - T0_Vicon;    % get time difference
    Nsamp = floor(alignflag*(rescaleFlagVicon + floor(abs(dT)/TsVicon*Ts)));  % get number of samples to pad
    if dT < 0 
        data.p = [data.p(1,:).*ones(Nsamp,3); data.p];
    else
        data.p(1:Nsamp,:) = [];
    end

    % get data
    tmp = [];
    for i=1:size(data.p,2)
        tmp(:,i) = resample(data.p(:,i),numel(time_full),size(data.p,1));  
    end      
    data.p = tmp(1:pos,:);

    %% distances

    % plot flag 
    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
    end
    
    % get data for UWB distances
    xhat = cell2mat(cellfun(@(m) double(m.DC),out.UWBData,'UniformOutput',false));    

    % reshape with number of tags and anchors
    xhat = reshape(xhat,12,length(out.UWBData))';

    % get time offset
    T0_UWB = double(out.UWBData{1}.Header.Stamp.Sec);   % get T0 for UWB
    TsUWB = Ts; %mean(diff(cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),out.UWBData,'UniformOutput',false))));
    dT = T0_GT - T0_UWB;    % get time difference
    Nsamp = floor(alignflag*(rescaleFlagUWB + abs(dT)/TsUWB*Ts));  % get number of samples to pad
    if dT < 0 
        xhat = [xhat(1,:).*ones(Nsamp,12); xhat];
    else
        xhat(1:Nsamp,:) = [];
    end

    % resample on the GT samples
    tmp = [];
    for i=1:size(xhat,2)
        tmp(:,i) = resample(xhat(:,i),numel(time_full),size(xhat,1));
    end
    xhat = tmp(1:pos,:);
    data.UWB = xhat;

    % plot distances
    if plotF
        sgtitle('Tag distances')
        ax = zeros(1,3);
        for i=1:3
            subplot(3,1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(3,1,i);     
            
            
            plot(time,data.UWB(:,4*(i-1)+1),'LineWidth',2,'Color','r');
            plot(time,data.UWB(:,4*(i-1)+2),'LineWidth',2,'Color','g');
            plot(time,data.UWB(:,4*(i-1)+3),'LineWidth',2,'Color','b');
            plot(time,data.UWB(:,4*(i-1)+4),'LineWidth',2,'Color','k');
                
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['DT_',num2str(i)])
        end
        legend('UWB MEAS A1','UWB MEAS A2','UWB MEAS A3','UWB MEAS A4')   
        xlabel('time [s]') 
    end

    %% position estimation

    % plot flag
    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
    end

    % get EKF
    xhat = [];
    xhat(:,1) = cellfun(@(m) double(m.Pose.Pose.Position.X),out.EKFData); 
    xhat(:,2) = cellfun(@(m) double(m.Pose.Pose.Position.Y),out.EKFData);
    xhat(:,3) = cellfun(@(m) double(m.Pose.Pose.Position.Z),out.EKFData); 

    % starting time for EKF 
    T0_EKF = double(out.EKFData{1}.Header.Stamp.Sec);   % get T0 for EKF
    TsEKF = Ts; %mean(diff(cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),out.EKFData,'UniformOutput',false))));
    dT = T0_GT - T0_EKF;    % get time difference
    Nsamp = floor(alignflag*(rescaleFlagEKF + abs(dT)/TsEKF*Ts));  % get number of samples to pad
    if dT < 0 
        xhat = [xhat(1,:).*ones(Nsamp,3); xhat];
    else
        xhat(1:Nsamp,:) = [];
    end
    
    % get data
    tmp = [];
    for i=1:size(xhat,2)
        tmp(:,i) = resample(xhat(:,i),numel(time_full),size(xhat,1));  
    end
    xhat = tmp(1:pos,:);
    data.phat = xhat;    

    % get PJUMP
    xhat = [];
    xhat(:,1) = cellfun(@(m) double(m.Pose.Pose.Position.X),out.PJUMPData); 
    xhat(:,2) = cellfun(@(m) double(m.Pose.Pose.Position.Y),out.PJUMPData);
    xhat(:,3) = cellfun(@(m) double(m.Pose.Pose.Position.Z),out.PJUMPData); 

    % starting time for EKF 
    T0_PJUMP = double(out.PJUMPData{1}.Header.Stamp.Sec);   % get T0 for PJUMP
    TsPJUMP = Ts; %mean(diff(cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),out.EKFData,'UniformOutput',false))));
    dT = T0_GT - T0_PJUMP;    % get time difference
    Nsamp = floor(alignflag*(rescaleFlagPJUMP + abs(dT)/TsPJUMP*Ts));  % get number of samples to pad
    if dT < 0 
        xhat = [xhat(1,:).*ones(Nsamp,3); xhat];
    else
        xhat(1:Nsamp,:) = [];
    end
    
    % get data
    tmp = [];
    for i=1:size(xhat,2)
        tmp(:,i) = resample(xhat(:,i),numel(time_full),size(xhat,1));  
    end
    xhat = tmp(1:pos,:);
    data.pjump = xhat;    

    % plot all these data
    if plotF
        for i=1:3
            subplot(3,1,i);
            box on
            hold on
            grid on
            plot(time,data.p(:,i),'b','LineWidth',2);
            plot(time,data.phat(:,i),'r','LineWidth',2);
            plot(time,data.pjump(:,i),'k','LineWidth',2);
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['p_',num2str(i)])
        end
        
        legend('Vicon','EKF','Jump')   
        xlabel('time [s]')
        xlim('auto');
    end

    %% pose estimation

    % plot flag
    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
    end

    % define ground truth
    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Orientation.X),out.viconData);
    else        
        x = zeros(numel(time),1);
    end
    data.q(:,2) = x;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),out.viconData);
    else        
        x = zeros(numel(time),1);
    end
    data.q(:,3) = x;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),out.viconData);
    else        
        x = zeros(numel(time),1);
    end
    data.q(:,4) = x;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Orientation.W),out.viconData);
    else        
        x = zeros(numel(time),1);
    end
    data.q(:,1) = x;

    % starting time for Vicon 
    T0_Vicon = double(out.viconData{1}.Header.Stamp.Sec);   % get T0 for Vicon
    dT = T0_GT - T0_Vicon;    % get time difference
    Nsamp = floor(alignflag*(rescaleFlagVicon + abs(dT)/TsVicon*Ts));  % get number of samples to pad
    if dT < 0 
        data.q = [data.q(1,:).*ones(Nsamp,4); data.q];
    else
        data.q(1:Nsamp,:) = [];
    end

    % get data
    tmp = [];
    for i=1:size(data.q,2)
        tmp(:,i) = resample(data.q(:,i),numel(time_full),size(data.q,1));  
    end
    data.q = tmp(1:pos,:);

    % get orientation from EKF
    xhat = [];
    xhat(:,2) = cellfun(@(m) double(m.Pose.Pose.Orientation.X),out.EKFData);
    xhat(:,3) = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),out.EKFData);
    xhat(:,4) = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),out.EKFData);
    xhat(:,1) = cellfun(@(m) double(m.Pose.Pose.Orientation.W),out.EKFData);

    % starting time for EKF 
    T0_EKF = double(out.EKFData{1}.Header.Stamp.Sec);   % get T0 for EKF
    dT = T0_GT - T0_EKF;    % get time difference
    Nsamp = floor(alignflag*(rescaleFlagEKF + abs(dT)/TsEKF*Ts));  % get number of samples to pad
    if dT < 0 
        xhat = [xhat(1,:).*ones(Nsamp,4); xhat];
    else
        xhat(1:Nsamp,:) = [];
    end
    
    % get data
    tmp = [];
    for i=1:size(xhat,2)
        tmp(:,i) = resample(xhat(:,i),numel(time_full),size(xhat,1));  
    end
    xhat = tmp(1:pos,:);
    data.qhat = xhat; 

    % get QJUMP
    xhat = [];
    xhat(:,2) = cellfun(@(m) double(m.Pose.Pose.Orientation.X),out.PJUMPData); 
    xhat(:,3) = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),out.PJUMPData);
    xhat(:,4) = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),out.PJUMPData);
    xhat(:,1) = cellfun(@(m) double(m.Pose.Pose.Orientation.W),out.PJUMPData);

    % starting time for EKF 
    T0_PJUMP = double(out.PJUMPData{1}.Header.Stamp.Sec);   % get T0 for PJUMP
    TsPJUMP = Ts; %mean(diff(cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),out.EKFData,'UniformOutput',false))));
    dT = T0_GT - T0_PJUMP;    % get time difference
    Nsamp = floor(alignflag*(rescaleFlagPJUMP + abs(dT)/TsPJUMP*Ts));  % get number of samples to pad
    if dT < 0 
        xhat = [xhat(1,:).*ones(Nsamp,4); xhat];
    else
        xhat(1:Nsamp,:) = [];
    end
    
    % get data
    tmp = [];
    for i=1:size(xhat,2)
        tmp(:,i) = resample(xhat(:,i),numel(time_full),size(xhat,1));  
    end
    xhat = tmp(1:pos,:);
    data.qjump = xhat;   

    % normalization
    data.q = quatnormalize(data.q);
    data.qhat = quatnormalize(data.qhat);

    % back to EUL angles
    [pitch, roll, yaw] = quat2angle( data.q, 'YXZ' );
    data.RPY(:,1) = roll;
    data.RPY(:,2) = pitch;
    data.RPY(:,3) = yaw;

    [pitch, roll, yaw] = quat2angle( data.qhat, 'YXZ' );
    data.RPYhat(:,1) = roll;
    data.RPYhat(:,2) = pitch;
    data.RPYhat(:,3) = yaw;

    if plotF
        for i=1:3
            subplot(3,1,i);
            box on
            hold on
            grid on
            plot(time,data.RPY(:,i),'b','LineWidth',2);
            plot(time,data.RPYhat(:,i),'r','LineWidth',2);
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['\theta_',num2str(i)])
        end
        
        legend('Vicon','EKF')   
        xlabel('time [s]') 
    end

    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
        for i=1:4
            subplot(4,1,i);
            box on
            hold on
            grid on
            plot(time,data.q(:,i),'b','LineWidth',2);
            plot(time,data.qhat(:,i),'r','LineWidth',2);
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['e_',num2str(i)])
        end
        
        legend('Vicon','EKF')      
        xlabel('time [s]') 
    end

    %% get IMU

    % get data acc
    xhat = [];
    xhat(:,1) = cellfun(@(m) double(m.LinearAcceleration.X),out.IMUData);
    xhat(:,2) = cellfun(@(m) double(m.LinearAcceleration.Y),out.IMUData);
    xhat(:,3) = cellfun(@(m) double(m.LinearAcceleration.Z),out.IMUData);

    % starting time for IMU 
    T0_IMU = double(out.IMUData{1}.Header.Stamp.Sec);   % get T0 for IMU
    TsIMU = Ts; %mean(diff(cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),out.IMUData,'UniformOutput',false))));
    dT = T0_GT - T0_IMU;    % get time difference
    Nsamp = floor(alignflag*(rescaleFlagIMU + abs(dT)/TsIMU*Ts));  % get number of samples to pad
    if dT < 0 
        xhat = [xhat(1,:).*ones(Nsamp,3); xhat];
    else
        xhat(1:Nsamp,:) = [];
    end

    % get data
    tmp = [];
    for i=1:size(xhat,2)
        tmp(:,i) = resample(xhat(:,i),numel(time_full),size(xhat,1));  
    end
    xhat = tmp(1:pos,:);
    data.IMU = xhat;

    
    % get data omega
    xhat = [];
    xhat(:,1) = cellfun(@(m) double(m.AngularVelocity.X),out.IMUData);
    xhat(:,2) = cellfun(@(m) double(m.AngularVelocity.Y),out.IMUData);
    xhat(:,3) = cellfun(@(m) double(m.AngularVelocity.Z),out.IMUData);
    
    % align
    if dT < 0 
        xhat = [xhat(1,:).*ones(Nsamp,3); xhat];
    else
        xhat(1:Nsamp,:) = [];
    end

    % get data
    tmp = [];
    for i=1:size(xhat,2)
        tmp(:,i) = resample(xhat(:,i),numel(time_full),size(xhat,1));  
    end
    xhat = tmp(1:pos,:);
    data.W = xhat;

    
    %% plotting everything
    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
    end
    
    if plotF
        sgtitle('IMU')
        ax = zeros(1,3);
        for i=1:3
            subplot(3,1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(3,1,i);     
            
            
            plot(time,data.IMU(:,i),'LineWidth',2,'Color','r');
            
                
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['a_',num2str(i)])
        end
        legend('Meas')   
        xlabel('time [s]') 
    end
    
    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
        sgtitle('OMEGA')
        ax = zeros(1,3);
        for i=1:3
            subplot(3,1,i);
            hold on
            grid on
            box on
    
            % indicize axes        
            ax(i)=subplot(3,1,i);     
            
            
            plot(time,data.W(:,i),'LineWidth',2,'Color','r');
            
                
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['w_',num2str(i)])
        end
        legend('Meas')   
        xlabel('time [s]') 
    end
   

end