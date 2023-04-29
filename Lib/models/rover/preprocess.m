%% preprocess data from rover acquisition

function out = preprocess(filename,ID)

    % load results from .mat
    load(filename,'test');
    topics = test.topics;

    %%% get ground truth %%%
    out.Pp1 = test.groundtruth.data.measure_1;
    out.Pp2 = test.groundtruth.data.measure_2;

    % trilateration
    out.PolesPos = [-1.0 +3.5 -3.5; ...
                    +3.5 -3.5 -3.5; ...
                    +1.7 +1.7 +1.7];

    % fake
    % out.PolesPos = [+0.0 +0.0 +2.0; ...
    %                 +2.0 -2.0 -0.0; ...
    %                 +2.2 +2.2 +2.2];
    % out.Pp1 = [2 2 2];
    dist_optoptions = optimoptions('fminunc', 'MaxIter', 10, 'display','off');
    x0 = [10; 10; 2.2];
    for i=1:size(out.Pp1,1)
        out.GT1(:,i) = fminunc(@(x)J_dist(x,out.PolesPos,out.Pp1(i,:)),x0,dist_optoptions);
        %GT2p(:,i) = fminunc(@(x)J_dist(x,out.PolesPos,out.Pp2(i,:)),zeros(3,1),dist_optoptions);
    end
    
    % TRANSFORMATION
    out.PolesTime = test.groundtruth.times - topics{ID(1)}.times(1);

    


    %%%%% IMU %%%%%
    topid = ID(1);
    % get IMU
    IMU_raw(:,1) = double(topics{topid}.data.linear_acc.vector.x);
    IMU_raw(:,2) = double(topics{topid}.data.linear_acc.vector.y);
    IMU_raw(:,3) = double(topics{topid}.data.linear_acc.vector.z);

    % sampling times - IMU
    IMUt = 0.01;

    % time intervals - IMU
    IMUtime_raw = double(topics{topid}.times - topics{topid}.times(1));
    IMUtspan = double([topics{topid}.times(1) topics{topid}.times(end)] - topics{topid}.times(1));

    % resample and interp IMU at 100Hz
    IMUtime_resamp = (IMUtspan(1):IMUt:IMUtspan(2)).';
    IMU_resamp = interp1(IMUtime_raw,IMU_raw,IMUtime_resamp);

    %%%%% UWB %%%%%
    % get UWB
    topid = ID(2);
    UWB_raw = double(topics{topid}.data.anchors.distances);

    % get number of anchors
    Nanchors = size(UWB_raw,2);

    % get anchor position
    out.AM1(1,:) = double(mean(topics{topid}.data.anchors.position.x,1));
    out.AM1(2,:) = double(mean(topics{topid}.data.anchors.position.y,1));
    out.AM1(3,:) = double(mean(topics{topid}.data.anchors.position.z,1)) + 2.20;

    % sampling times - UWB
    UWBt = 0.2;

    % time intervals - UWB
    UWBtime_raw = double(topics{topid}.times - topics{topid}.times(1));
    UWBtspan = double([topics{topid}.times(1) topics{topid}.times(end)] - topics{topid}.times(1));

    % resample and interp IMU at 100Hz
    UWBtime_resamp = (UWBtspan(1):UWBt:UWBtspan(2)).';
    UWB_resamp = interp1(UWBtime_raw,UWB_raw,UWBtime_resamp);
    
    % make the two arrays consistent
    tscale = UWBt/IMUt;
    IMU_expected = size(UWB_resamp,1)*tscale;    
    tmp = size(IMU_resamp,1);
    if tmp > IMU_expected
        IMU_resamp = IMU_resamp(1:IMU_expected,:);
        IMUtime_resamp = IMUtime_resamp(1:IMU_expected);
    else
        IMU_resamp(tmp+1:IMU_expected,:) = repmat(IMU_resamp(end,:),IMU_expected-tmp,1);
        IMUtime_resamp(tmp+1:IMU_expected) = IMUtime_resamp(tmp) + (IMUt:IMUt:(IMU_expected-tmp)*IMUt).';
    end    

    % define filter IMU
    order = 2;
    [B,A] = butter(order,(2*pi*0.05)/(IMUt),'s');
    F = tf(B,A);
    F_IMU = c2d(F,IMUt); 

    % filter IMU
    for i=1:size(IMU_resamp,2)
        IMU(:,i) = lsim(F_IMU,IMU_resamp(:,i),IMUtime_resamp,IMU_resamp(1:order,i));
    end

    % define filter UWB
    order = 2;
    [B,A] = butter(order,(2*pi*0.1)/(UWBt),'s');
    F = tf(B,A);
    F_UWB = c2d(F,UWBt); 

    % filter IMU
    for i=1:size(UWB_resamp,2)
        UWB(:,i) = lsim(F_UWB,UWB_resamp(:,i),UWBtime_resamp,UWB_resamp(1:order,i));
    end

    % save vars
    out.IMU_raw = IMU_raw;
    out.UWB_raw = UWB_raw;
    out.IMU_resamp = IMU_resamp;
    out.UWB_resamp = UWB_resamp;
    out.IMU = IMU;
    out.UWB = UWB;
    out.IMUtime_raw = IMUtime_raw;
    out.UWBtime_raw = UWBtime_raw;
    out.IMUtime_resamp = IMUtime_resamp;
    out.UWBtime_resamp = UWBtime_resamp;
    out.IMUt = IMUt;
    out.UWBt = UWBt;
    out.tscale = tscale;
    out.F_IMU = F_IMU;
    out.F_UWB = F_UWB;
    out.Nanchors = Nanchors;

end
