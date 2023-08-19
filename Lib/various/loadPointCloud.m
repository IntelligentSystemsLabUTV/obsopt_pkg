%% visualize pointcloud

function out = loadPointCloud(filename,plotFlag,roi)

    out.cloud = e57FileReader(filename);

    % Define variables to store point clouds and their corresponding poses
    ptCloudArr = [];
    tformArr   = [];
    
    % Read point cloud data
    [ptCloud, pcMetadata] = readPointCloud(out.cloud,1);

    ptCloudArr  = ptCloud(1);
    tformArr = pcMetadata.RelativePose;

    pcAlign = pcalign(ptCloudArr,tformArr);
    tmp = pcAlign.Location;
    tmp = reshape(tmp,size(ptCloud.Location));

    % set limits
    IDX = (tmp(:,:,1) > roi(1,1) & tmp(:,:,1) < roi(1,2));
    IDY = (tmp(:,:,2) > roi(2,1) & tmp(:,:,2) < roi(2,2));
    IDZ = (tmp(:,:,3) > roi(3,1) & tmp(:,:,3) < roi(3,2));
    ID = IDX & IDY & IDZ;
    pcAlignSel = select(ptCloud, ID);

    tformArrInv = tformArr.invert;
    tmpMap = pcalign(pcAlignSel,tformArr);

    % set the origin
    point = 1*[-5.10, + 6.98, - 1.31];
    translatedLocations = tmpMap.Location - point;
    translatedColors = tmpMap.Color;
    tmpMap = pointCloud(translatedLocations, 'Color', translatedColors);

    % rotation matrix
    R = eul2rotm([0 0 pi/2],'XYZ');
    rotatedLocations = (R * tmpMap.Location')';
    translatedColors = tmpMap.Color;
    out.ptMap = pointCloud(rotatedLocations, 'Color', translatedColors);
    
    % Visualize the map
    if plotFlag
        figure
        hold on
        pcshow(out.ptMap);
    end

    out.GTF = [ 1.94 0.92 -0.86; 
                2.94 0.93 -0.84;
                4.08 0.95 -0.84;
                5.02 0.95 -0.85;
                7.65 0.93 -1.03;
                9.00 0.94 -1.17;
                10.33 0.91 -1.22;
                11.38 0.94 -1.27;
                11.11 3.72 -1.23;
                10.11 3.73 -1.22;
                8.78  3.75 -1.17;
                5.04  3.72 -0.83;
                4.03  3.74 -0.86;
                3.04  3.75 -0.87;
                2.06  3.73 -0.90;
                1.05  3.74 -0.93    ];    

end