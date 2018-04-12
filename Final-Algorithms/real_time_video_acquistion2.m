clc;
clear;
close all;

videoPlayer = vision.VideoPlayer;
vid = videoinput('winvideo',1,'MJPG_544x288');
vid.TriggerRepeat = Inf;
vid.FramesPerTrigger=1;

%Initialization
FMV=[0 0];
constant_motion_vector_I=[0 0];
global_motion_vector=[0 0];
Hcumulative = (eye(3));

% Start acquiring frames.
start(vid)
% Calculate difference image and display it.
while(vid.FramesAvailable >= 0)
    data = getdata(vid,2);      
    prev_frame=rgb2gray(data(:,:,:,2));
    current_frame=rgb2gray(data(:,:,:,1));     
    %***************************************************************************
    ptThresh = 0.1;
    pointsA = detectFASTFeatures(prev_frame, 'MinContrast', ptThresh,'MinQuality',0.3);
    pointsB = detectFASTFeatures(current_frame, 'MinContrast', ptThresh,'MinQuality',0.3);
    %***************************************************************************
    [featuresA, pointsA] = extractFeatures(prev_frame,(pointsA));
    [featuresB, pointsB] = extractFeatures(current_frame, (pointsB));
    indexPairs = matchFeatures(featuresA, featuresB,'Method','Approximate');
    pointsA = pointsA(indexPairs(:, 1), :);
    pointsB = pointsB(indexPairs(:, 2), :);
    
    [tform, pointsBm, pointsAm] = estimateGeometricTransform(pointsB, pointsA, 'affine','MaxNumTrials',200);
    
    % Extract scale and rotation part sub-matrix.
    H = tform.T;
    R = H(1:2,1:2);
    % Compute theta from mean of two possible arctangents
    theta = mean([atan2(R(2),R(1)) atan2(-R(3),R(4))]);
    % Compute scale from mean of two stable mean calculations
%   scale = mean(R([1 4])/cos(theta));
    scale=1;
    % Translation remains the same:
    translation = (H(3, 1:2));
    % Reconstitute new s-R-t transform:
    HsRt =( [[scale*[cos(theta) -sin(theta); sin(theta) cos(theta)]; ...
      0 0], [0 0 1]']);
    Hcumulative = HsRt * Hcumulative;    
    current_frame_no_rotation = imwarp(current_frame,affine2d(Hcumulative),'OutputView',imref2d(size(current_frame)));
    global_motion_vector_prev=global_motion_vector;
    global_motion_vector=[translation(2),translation(1)];
     %Correction
    
    k=0.95 ;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)%TWEAKABLE PARAMETER
    beta=0.015;%0.015%TWEAKABLE PARAMETER
    alpha=1;%TWEAKABLE PARAMETER (between 0 and 1)
    gamma=1;%TWEAKABLE PARAMETER (between 0 and 1)
    FMV=round(k*FMV+...
    (gamma*(alpha*global_motion_vector+(1-alpha)*global_motion_vector_prev))+...
    ((1-gamma)*(global_motion_vector-beta*constant_motion_vector_I)));
    constant_motion_vector_I=constant_motion_vector_I+FMV;    
    frame_crop_size=100;
    if FMV(1)>frame_crop_size
        FMV(1)=frame_crop_size;
    end
    if FMV(1)<-frame_crop_size
        FMV(1)=-frame_crop_size;
    end
    if FMV(2)>frame_crop_size
        FMV(2)=frame_crop_size;
    end
    if FMV(2)<-frame_crop_size
        FMV(2)=-frame_crop_size;
    end    
    stabilized_frame=current_frame_no_rotation((frame_crop_size+1:(size(current_frame_no_rotation,1)-frame_crop_size))-FMV(1),(frame_crop_size+1:(size(current_frame_no_rotation,2)-frame_crop_size))-FMV(2));
    temp_frame= current_frame((frame_crop_size+1:(size(current_frame,1)-frame_crop_size)),(frame_crop_size+1:(size(current_frame,2)-frame_crop_size)));    
    
    step(videoPlayer, [temp_frame  stabilized_frame]); 
    
end
stop(vid);

