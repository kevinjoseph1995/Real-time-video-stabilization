clc;
clear;
close all;
%***************************************************************************
% Video Write-to-file initialization
stabilized_video_prototype = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\optimized_prototype_2.mp4','MPEG-4');
stabilized_video_prototype.FrameRate = 30;%TWEAKABLE PARAMETER
open(stabilized_video_prototype);

%Initialization
FMV=[0 0];
constant_motion_vector_I=[0 0];

%***************************************************************************
%Video Player Initialization
videoPlayer = vision.VideoPlayer;
%***************************************************************************

%Video Read Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\web_video_4.mp4';
filename = 'C:\Users\Kevin Joseph\Desktop\robot_video.avi';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_2.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
%***************************************************************************
%Loop Initialization

prev_frame=step(hVideoSrc);
frame_no=1;
global_motion_vector=[0 0];
Hcumulative = (eye(3));
while ~isDone(hVideoSrc) 
    %Reading current frame
    current_frame=step(hVideoSrc);
    prev_frame_cpu=(prev_frame);
    current_frame_cpu=(current_frame);
    %***************************************************************************
    ptThresh = 0.1;
    pointsA = detectFASTFeatures(prev_frame, 'MinContrast', ptThresh,'MinQuality',0.3);
    pointsB = detectFASTFeatures(current_frame, 'MinContrast', ptThresh,'MinQuality',0.3);
    %***************************************************************************
    [featuresA, pointsA] = extractFeatures(prev_frame_cpu,(pointsA));
    [featuresB, pointsB] = extractFeatures(current_frame_cpu, (pointsB));
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
    current_frame_no_rotation = imwarp(current_frame_cpu,affine2d(Hcumulative),'OutputView',imref2d(size(current_frame_cpu)));
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
%     temp_frame= current_frame((frame_crop_size+1:(size(current_frame,1)-frame_crop_size)),(frame_crop_size+1:(size(current_frame,2)-frame_crop_size))); 
    
    writeVideo(stabilized_video_prototype,(stabilized_frame));
    step(videoPlayer, (stabilized_frame));    
    prev_frame=current_frame;
    frame_no=frame_no+1;

end
% Here you call the release method on the objects to close any open files
% and release memory.
release(hVideoSrc);
close(stabilized_video_prototype);
release(videoPlayer);