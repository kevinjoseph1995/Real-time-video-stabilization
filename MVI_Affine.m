clc;
clear;
close all;
%***************************************************************************
%Video Write-to-file initialization
stabilized_video_prototype = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\mvi_affine.mp4','MPEG-4');
stabilized_video_prototype.FrameRate = 30;
open(stabilized_video_prototype);
%Initialization
constant_motion_vector=gpuArray([0 0]);
constant_motion_vector_I=gpuArray([0 0]);
%***************************************************************************
%Video Read Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\web_video_4.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_2.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
%***************************************************************************
%Loop Initialization

prev_frame=gpuArray(step(hVideoSrc));
Hcumulative = eye(3);
i=1;
while ~isDone(hVideoSrc)  && i<1000
    current_frame=gpuArray(step(hVideoSrc));
    
    %***************************************************************************
    ptThresh = 0.2;
    pointsA = detectFASTFeatures(prev_frame, 'MinContrast', ptThresh);
    pointsB = detectFASTFeatures(current_frame, 'MinContrast', ptThresh);
    %***************************************************************************
    [featuresA, pointsA] = extractFeatures(gather(prev_frame),gather(pointsA));
    [featuresB, pointsB] = extractFeatures(gather(current_frame), gather(pointsB));
    indexPairs = matchFeatures(featuresA, featuresB,'Method','Approximate');
    pointsA = pointsA(indexPairs(:, 1), :);
    pointsB = pointsB(indexPairs(:, 2), :);
    
    if pointsA.Count<3 || pointsB.Count<3
        crop_factor=200;
        current_frame_cropped=current_frame(crop_factor+1:size(current_frame,1) ...
        -crop_factor,crop_factor+1:size(current_frame,2)-crop_factor);
        new_frame=[current_frame_cropped  current_frame_cropped];
        new_frame=mat2gray(imresize(new_frame,0.8));
        writeVideo(stabilized_video_prototype,gather(new_frame));
        prev_frame=current_frame;       
        
        continue;
    end

    
    [tform, pointsBm, pointsAm] = estimateGeometricTransform(pointsB, pointsA, 'affine');
    
    % Extract scale and rotation part sub-matrix.
    H = tform.T;
    R = H(1:2,1:2);
    % Compute theta from mean of two possible arctangents
    theta = mean([atan2(R(2),R(1)) atan2(-R(3),R(4))]);
    % Compute scale from mean of two stable mean calculations
%   scale = mean(R([1 4])/cos(theta));
    scale=1;
    % Translation remains the same:
    translation = gpuArray(H(3, 1:2));
    % Reconstitute new s-R-t transform:
    HsRt = [[scale*[cos(theta) -sin(theta); sin(theta) cos(theta)]; ...
      0 0], [0 0 1]'];
    Hcumulative = HsRt * Hcumulative;    
    current_frame_no_rotation = imwarp(gather(current_frame),affine2d(Hcumulative),'OutputView',imref2d(size(gather(current_frame))));
    %**************************************************
    
  
    
    k=0.95 ;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)
    beta=0.015;%0.015
    translation=round(translation);
    constant_motion_vector=round(k*constant_motion_vector+[translation(2),translation(1)]-beta*constant_motion_vector_I);
    constant_motion_vector_I=constant_motion_vector_I+constant_motion_vector;
    
    if norm(gather(constant_motion_vector))>250
        prev_frame=current_frame;
        continue;
    end
    
    crop_factor=250;
    current_frame_stabilized=current_frame_no_rotation((crop_factor+1:size(current_frame_no_rotation,1)-crop_factor)...
    -constant_motion_vector(1),(crop_factor+1:size(current_frame_no_rotation,2)-crop_factor)-constant_motion_vector(2));
    current_frame_cropped=current_frame(crop_factor+1:size(current_frame,1)-crop_factor,crop_factor+1:size(current_frame,2)-crop_factor);
    
    
    %**************************************************
    
    writeVideo(stabilized_video_prototype,gather(current_frame_stabilized));
    
    prev_frame=current_frame;
    i=i+1
end
close(stabilized_video_prototype);
% Here you call the release method on the objects to close any open files
% and release memory.
release(hVideoSrc);