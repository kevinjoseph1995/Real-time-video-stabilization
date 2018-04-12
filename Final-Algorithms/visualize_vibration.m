clc;
clear;
close all;
% ***************************************************************************
% Video Write-to-file initialization
stabilized_video_prototype = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\vibration_visualized_algo2.mp4','MPEG-4');
stabilized_video_prototype.FrameRate = 30;%TWEAKABLE PARAMETER
open(stabilized_video_prototype);

%Initialization
%Video Read Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\optimized_prototype_2.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_4.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');

%***************************************************************************
%Loop Initialization

videoPlayer = vision.VideoPlayer;
opticFlow = opticalFlowLK();
frame_no=1;
count=1;
I=uint8(zeros(100,100,3,16));
while ~isDone(hVideoSrc) 
    %Reading current frame
    frame =(step(hVideoSrc));
    crop_factor=50;
    frame=frame((crop_factor+1:size(frame,1)-crop_factor),(crop_factor+1:size(frame,2)-crop_factor));    
    flow = estimateFlow(opticFlow,frame);
    f=figure(1);
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
    imshow(frame)
    hold on
    plot(flow,'DecimationFactor',[30 30],'ScaleFactor',15)
    hold off
    set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    F = getframe(f);
    writeVideo(stabilized_video_prototype,F.cdata); 
    if frame_no>=101&&frame_no<=116
        I(:,:,:,count)=uint8(F.cdata(51+100:150+100,161+100:260+100,:));
        count=count+1;
    end
    frame_no=frame_no+1
   
   
end
figure(2);
montage(I);
title('Algorithm 2 Section Montage');
close(stabilized_video_prototype);
% Here you call the release method on the objects to close any open files
% and release memory.
release(videoPlayer);
release(hVideoSrc);
%release(videoPlayer);