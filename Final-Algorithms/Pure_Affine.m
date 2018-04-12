clc;
clear;
close all;
%***************************************************************************
%Video Write-to-file initialization
video_write_obj = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\pure_affine.mp4','MPEG-4');
video_write_obj.FrameRate = 30;
open(video_write_obj);
%***************************************************************************
%Video Read-from-file Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\web_video_4.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_2.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
%***************************************************************************
%Other Initializations

frame_no=1;
prev_frame=gpuArray(step(hVideoSrc));
crop_factor=50;
prev_frame=prev_frame(crop_factor+1:size(prev_frame,1)-crop_factor,crop_factor+1 ...
:size(prev_frame,2)-crop_factor);
Hcumulative = eye(3);
while ~isDone(hVideoSrc)
    frame=gpuArray(step(hVideoSrc));
    
    crop_factor=50;
    frame=frame(crop_factor+1:size(frame,1)-crop_factor, ...
    crop_factor+1:size(frame,2)-crop_factor);
    
    %********************
    ptThresh = 0.1;
    pointsA = detectFASTFeatures(prev_frame, 'MinContrast', ptThresh);
    pointsB = detectFASTFeatures(frame, 'MinContrast', ptThresh);
    % Extract FREAK descriptors for the corners
    [featuresA, pointsA] = extractFeatures(gather(prev_frame), gather(pointsA));
    [featuresB, pointsB] = extractFeatures(gather(frame), gather(pointsB));
    indexPairs = matchFeatures(featuresA, featuresB,'Method','Approximate');
    pointsA = pointsA(indexPairs(:, 1), :);
    pointsB = pointsB(indexPairs(:, 2), :);
    [tform, pointsBm, pointsAm] = estimateGeometricTransform(...
    pointsB, pointsA, 'affine');
    H = tform.T;
    R = H(1:2,1:2);
    % Compute theta from mean of two possible arctangents
    theta = mean([atan2(R(2),R(1)) atan2(-R(3),R(4))]);
    % Compute scale from mean of two stable mean calculations
    scale = mean(R([1 4])/cos(theta));
    % Translation remains the same:
    translation = H(3, 1:2);
    % Reconstitute new s-R-t transform:
    HsRt = [[scale*[cos(theta) -sin(theta); sin(theta) cos(theta)]; ...
      translation], [0 0 1]'];
    Hcumulative = HsRt * Hcumulative;
    stabilized_frame = imwarp(gather(frame),affine2d(Hcumulative),'OutputView',imref2d(size(gather(frame))));
    writeVideo(video_write_obj,stabilized_frame);
    prev_frame=frame;
    frame_no=frame_no+1
end
close(video_write_obj);
release(hVideoSrc);