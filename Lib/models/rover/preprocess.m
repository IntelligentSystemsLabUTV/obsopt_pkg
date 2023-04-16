%% preprocess data from rover acquisition

function out = preprocess(filename)

    % load results from .mat
    load(filename,'topics');

    %%%%% IMU %%%%%
    % get IMU
    IMU_raw(:,1) = topics{4}.data.linear_acc.vector.x;
    IMU_raw(:,2) = topics{4}.data.linear_acc.vector.y;
    IMU_raw(:,3) = topics{4}.data.linear_acc.vector.z;

    % sampling times - IMU
    IMUt = 0.01;

    % time intervals - IMU
    IMUtime_raw = topics{4}.times - topics{4}.times(1);
    IMUtspan = [topics{4}.times(1) topics{4}.times(end)] - topics{4}.times(1);

    % resample and interp IMU at 100Hz
    IMUtime_resamp = (IMUtspan(1):IMUt:IMUtspan(2)).';
    IMU_resamp = interp1(IMUtime_raw,IMU_raw,IMUtime_resamp);

    %%%%% UWB %%%%%
    % get UWB
    UWB_raw = topics{7}.data.anchors.distances;

    % sampling times - UWB
    UWBt = 0.2;

    % time intervals - UWB
    UWBtime_raw = topics{7}.times - topics{7}.times(1);
    UWBtspan = [topics{7}.times(1) topics{7}.times(end)] - topics{7}.times(1);

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
    [B,A] = butter(order,(2*pi)/(5*IMUt),'s');
    F = tf(B,A);
    F_IMU = c2d(F,IMUt); 

    % filter IMU
    for i=1:size(IMU_resamp,2)
        IMU(:,i) = lsim(F_IMU,IMU_resamp(:,i),IMUtime_resamp,IMU_resamp(1:order,i));
    end

    % define filter UWB
    order = 2;
    [B,A] = butter(order,(2*pi)/(5*UWBt),'s');
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

end
