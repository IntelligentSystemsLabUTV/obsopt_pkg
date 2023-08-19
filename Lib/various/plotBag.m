%% plot data
function data = plotBag(out,plotF,GTF,tstop,Ts)

    % init
    fig_count = 0;
    fontsize = 15;
    % close all
    set(0,'DefaultFigureWindowStyle','docked');

    % define time ofr gorund truth and estimation    
    if GTF
        time =  out.vicon.MessageList.Time - out.vicon.StartTime;
    else
        time =  out.EKF.MessageList.Time - out.EKF.StartTime;
    end
    time = time(1):Ts:time(end); 

    if isempty(tstop)
        val = time(end-1);
    else
        val = tstop;
    end
    [~, pos] = min(abs(time - val));  
    time = time(1:pos);   
    data.time = time;
    startpos = floor(numel(time)/2);
    endpos = numel(time)-1;

    %% position estimation
    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
    end

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Position.X),out.viconData);
        x = resample(x,numel(time),numel(x));
        x = x(1:pos);
    else        
        x = zeros(numel(time),1);
    end    
    data.p(:,1) = x;
    xhat = cellfun(@(m) double(m.Pose.Pose.Position.X),out.EKFData);       
    xhat = resample(xhat,size(data.p,1),numel(xhat));    
    xhat = xhat(1:pos);
    data.phat(:,1) = xhat;    

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Position.Y),out.viconData);
        x = resample(x,numel(time),numel(x));
        x = x(1:pos);
    else        
        x = zeros(numel(time),1);
    end
    data.p(:,2) = x;
    xhat = cellfun(@(m) double(m.Pose.Pose.Position.Y),out.EKFData);
    xhat = resample(xhat,size(data.p,1),numel(xhat));   
    xhat = xhat(1:pos); 
    data.phat(:,2) = xhat;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Position.Z),out.viconData);
        x = resample(x,numel(time),numel(x));
        x = x(1:pos);
    else        
        x = zeros(numel(time),1);
    end
    data.p(:,3) = x;
    xhat = cellfun(@(m) double(m.Pose.Pose.Position.Z),out.EKFData); 
    xhat = resample(xhat,size(data.p,1),numel(xhat));    
    xhat = xhat(1:pos);
    data.phat(:,3) = xhat;

    x = cellfun(@(m) double(m.Pose.Pose.Position.X),out.PJUMPData);
    x = resample(x,size(data.p,1),numel(x));
    data.pjump(:,1) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Position.Y),out.PJUMPData);
    x = resample(x,size(data.p,1),numel(x));
    data.pjump(:,2) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Position.Z),out.PJUMPData);
    x = resample(x,size(data.p,1),numel(x));
    data.pjump(:,3) = x;

    x = cellfun(@(m) double(m.Pose.Pose.Position.X),out.PHYBData);
    x = resample(x,size(data.p,1),numel(x));
    x = x(1:pos);
    data.phyb(:,1) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Position.Y),out.PHYBData);
    x = resample(x,size(data.p,1),numel(x));
    x = x(1:pos);
    data.phyb(:,2) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Position.Z),out.PHYBData);
    x = resample(x,size(data.p,1),numel(x));
    x = x(1:pos);
    data.phyb(:,3) = x;

    % movmean on EKF
    win = 200;
    for i=1:3
        data.phatfilt(:,i) = movmean(data.phat(:,i),win);
    end

    if plotF
        for i=1:3
            subplot(3,1,i);
            box on
            hold on
            grid on
            plot(time,data.p(:,i),'b','LineWidth',2);
            plot(time,data.phat(:,i),'r','LineWidth',2);
            plot(time,data.phyb(:,i),'k','LineWidth',2);
            plot(time,data.phatfilt(:,i),'g','LineWidth',2);
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['p_',num2str(i)])
        end
        
        legend('Pos')   
        xlabel('time [s]')
        xlim('auto');
    end

    %% position estimation error
    if plotF && 0
        fig_count = fig_count +1;
        figure(fig_count)
    end

    data.ep = data.p - data.phat;
    data.epMean = mean(data.ep(startpos:endpos,:),1);
    data.epSigma = std(data.ep(startpos:endpos,:),0,1);

    if plotF && 0
        for i=1:3
            subplot(3,1,i);
            box on
            hold on
            grid on
            plot(time,data.ep(:,i),'b','LineWidth',2);
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['e_',num2str(i)])
        end
        
        legend('Err')   
        xlabel('time [s]') 
        xlim('auto');
    end

    %% pose estimation
    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
    end

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Orientation.X),out.viconData);
        x = resample(x,numel(time),numel(x));
        x = x(1:pos);
    else        
        x = zeros(numel(time),1);
    end
    data.q(:,2) = x;
    xhat = cellfun(@(m) double(m.Pose.Pose.Orientation.X),out.EKFData);
    xhat = resample(xhat,size(data.p,1),numel(xhat));
    xhat = xhat(1:pos);
    data.qhat(:,2) = xhat;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),out.viconData);
        x = resample(x,numel(time),numel(x));
        x = x(1:pos);
    else        
        x = zeros(numel(time),1);
    end
    data.q(:,3) = x;
    xhat = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),out.EKFData);
    xhat = resample(xhat,size(data.p,1),numel(xhat));
    xhat = xhat(1:pos);
    data.qhat(:,3) = xhat;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),out.viconData);
        x = resample(x,numel(time),numel(x));
        x = x(1:pos);
    else        
        x = zeros(numel(time),1);
    end
    data.q(:,4) = x;
    xhat = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),out.EKFData);
    xhat = resample(xhat,size(data.p,1),numel(xhat));
    xhat = xhat(1:pos);
    data.qhat(:,4) = xhat;

    if GTF
        x = cellfun(@(m) double(m.Pose.Pose.Orientation.W),out.viconData);
        x = resample(x,numel(time),numel(x));
        x = x(1:pos);
    else        
        x = zeros(numel(time),1);
    end
    data.q(:,1) = x;
    xhat = cellfun(@(m) double(m.Pose.Pose.Orientation.W),out.EKFData);
    xhat = resample(xhat,size(data.p,1),numel(xhat));
    xhat = xhat(1:pos);
    data.qhat(:,1) = xhat;

    x = cellfun(@(m) double(m.Pose.Pose.Orientation.X),out.PJUMPData);
    x = resample(x,size(data.p,1),numel(x));
    data.qjump(:,2) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),out.PJUMPData);
    x = resample(x,size(data.p,1),numel(x));
    data.qjump(:,3) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),out.PJUMPData);    
    x = resample(x,size(data.p,1),numel(x));
    data.qjump(:,4) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Orientation.W),out.PJUMPData);    
    x = resample(x,size(data.p,1),numel(x));
    data.qjump(:,1) = x;

    x = cellfun(@(m) double(m.Pose.Pose.Orientation.X),out.PHYBData);    
    x = resample(x,size(data.p,1),numel(x));
    data.qhyb(:,2) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),out.PHYBData);    
    x = resample(x,size(data.p,1),numel(x));
    data.qhyb(:,3) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),out.PHYBData);    
    x = resample(x,size(data.p,1),numel(x));
    data.qhyb(:,4) = x;
    x = cellfun(@(m) double(m.Pose.Pose.Orientation.W),out.PHYBData);    
    x = resample(x,size(data.p,1),numel(x));
    data.qhyb(:,1) = x;

    data.q = quatnormalize(data.q);
    data.qhat = quatnormalize(data.qhat);
    data.qjump = quatnormalize(data.qjump);
    data.qhyb = quatnormalize(data.qhyb);

    [pitch, roll, yaw] = quat2angle( data.q, 'YXZ' );
    data.RPY(:,1) = roll;
    data.RPY(:,2) = pitch;
    data.RPY(:,3) = yaw;

    [pitch, roll, yaw] = quat2angle( data.qhat, 'YXZ' );
    data.RPYhat(:,1) = roll;
    data.RPYhat(:,2) = pitch;
    data.RPYhat(:,3) = yaw;

    [pitch, roll, yaw] = quat2angle( data.qjump, 'YXZ' );
    data.RPYjump(:,1) = roll;
    data.RPYjump(:,2) = pitch;
    data.RPYjump(:,3) = yaw;

    [pitch, roll, yaw] = quat2angle( data.qhyb, 'YXZ' );
    data.RPYhyb(:,1) = roll;
    data.RPYhyb(:,2) = pitch;
    data.RPYhyb(:,3) = yaw;

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
        
        legend('Ang')   
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
        
        legend('Quat')   
        xlabel('time [s]') 
    end

    %% pose estimation error
    if plotF && 0
        fig_count = fig_count +1;
        figure(fig_count)
    end

    data.eang = data.RPY - data.RPYhat;
    data.eangMean = mean(data.eang(startpos:endpos,:),1);
    data.eangSigma = std(data.eang(startpos:endpos,:),0,1);

    if plotF && 0
        for i=1:3
            subplot(3,1,i);
            box on
            hold on
            grid on
            plot(time,data.eang(:,i),'b','LineWidth',2);
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['e_',num2str(i)])
        end
        
        legend('Ang Err')   
        xlabel('time [s]') 
    end

    %% quaternion estimation error
    if plotF && 0
        fig_count = fig_count +1;
        figure(fig_count)
    end

    data.equat = quatmultiply(data.q,data.qhat);
    data.eangMean = mean(data.equat(startpos:endpos,:),1);
    data.eangSigma = std(data.equat(startpos:endpos,:),0,1);

    if plotF && 0
        for i=1:4
            subplot(4,1,i);
            box on
            hold on
            grid on
            plot(time,data.equat(:,i),'b','LineWidth',2);
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['e_',num2str(i)])
        end
        
        legend('Quat Err')   
        xlabel('time [s]') 
    end

    %% distances
    if plotF
        fig_count = fig_count +1;
        figure(fig_count)
    end
    
    xhat = cell2mat(cellfun(@(m) double(m.DC),out.UWBData,'UniformOutput',false));        
    xhat = reshape(xhat,12,length(out.UWBData))';
    for i=1:size(xhat,2)
        tmp(:,i) = resample(xhat(:,i),size(data.p,1),size(xhat,1));
    end
    xhat = tmp;
    data.UWB = xhat;

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
        legend('Meas')   
        xlabel('time [s]') 
    end

    %% getIMU
    data.IMU(:,1) = cellfun(@(m) double(m.LinearAcceleration.X),out.IMUData);
    data.IMU(:,2) = cellfun(@(m) double(m.LinearAcceleration.Y),out.IMUData);
    data.IMU(:,3) = cellfun(@(m) double(m.LinearAcceleration.Z),out.IMUData);
    data.IMU = resample(data.IMU,size(data.p,1),numel(data.IMU(:,1)));
    data.IMU = data.IMU(1:pos,:);
    tmp = [];
    for i = 1:3
        tmp(:,i) = resample(data.IMU(:,i),size(data.p,1),size(data.IMU(:,i),1));
    end
    data.IMU = tmp;

    data.W(:,1) = cellfun(@(m) double(m.AngularVelocity.X),out.IMUData);
    data.W(:,2) = cellfun(@(m) double(m.AngularVelocity.Y),out.IMUData);
    data.W(:,3) = cellfun(@(m) double(m.AngularVelocity.Z),out.IMUData);
    data.W = resample(data.W,size(data.p,1),numel(data.W(:,1)));
    data.W = data.W(1:pos,:);
    for i = 1:3
        tmp(:,i) = resample(data.W(:,i),size(data.p,1),size(data.W(:,i),1));
    end
    data.W = tmp;

    data.IMUHYB(:,1) = cellfun(@(m) double(m.LinearAcceleration.X),out.IMUHYBData);
    data.IMUHYB(:,2) = cellfun(@(m) double(m.LinearAcceleration.Y),out.IMUHYBData);
    data.IMUHYB(:,3) = cellfun(@(m) double(m.LinearAcceleration.Z),out.IMUHYBData);
    data.IMUHYB = resample(data.IMUHYB,size(data.p,1),numel(data.IMUHYB(:,1)));
    data.IMUHYB = data.IMUHYB(1:pos,:);
    tmp = [];
    for i = 1:3
        tmp(:,i) = resample(data.IMUHYB(:,i),size(data.p,1),size(data.IMUHYB(:,i),1));
    end
    data.IMUHYB = tmp;

    data.WHYB(:,1) = cellfun(@(m) double(m.AngularVelocity.X),out.IMUHYBData);
    data.WHYB(:,2) = cellfun(@(m) double(m.AngularVelocity.Y),out.IMUHYBData);
    data.WHYB(:,3) = cellfun(@(m) double(m.AngularVelocity.Z),out.IMUHYBData);
    data.WHYB = resample(data.WHYB,size(data.p,1),numel(data.WHYB(:,1)));
    data.WHYB = data.WHYB(1:pos,:);
    for i = 1:3
        tmp(:,i) = resample(data.WHYB(:,i),size(data.p,1),size(data.WHYB(:,i),1));
    end
    data.WHYB = tmp;

    data.BIASHYB(:,1) = cellfun(@(m) double(m.LinearAcceleration.X),out.BiasHYBData);
    data.BIASHYB(:,2) = cellfun(@(m) double(m.LinearAcceleration.Y),out.BiasHYBData);
    data.BIASHYB(:,3) = cellfun(@(m) double(m.LinearAcceleration.Z),out.BiasHYBData);
    data.BIASHYB = resample(data.BIASHYB,size(data.p,1),numel(data.BIASHYB(:,1)));
    data.BIASHYB = data.BIASHYB(1:pos,:);
    tmp = [];
    for i = 1:3
        tmp(:,i) = resample(data.BIASHYB(:,i),size(data.p,1),size(data.BIASHYB(:,i),1));
    end
    data.BIASHYB = tmp;
    
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
            plot(time,data.IMUHYB(:,i),'LineWidth',2,'Color','k');
            
                
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
            plot(time,data.WHYB(:,i),'LineWidth',2,'Color','k');
            
                
            % labels
            set(gca,'fontsize', fontsize)         
            ylabel(['w_',num2str(i)])
        end
        legend('Meas')   
        xlabel('time [s]') 
    end
   

end