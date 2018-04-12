clc;
clear;
close all;
%***************************************************************************
%Video Write-to-file initialization
video_write = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\vibration_visualized_combined.mp4','MPEG-4');
video_write.FrameRate = 30;%TWEAKABLE PARAMETER
open(video_write);

%Initialization
%Video Read Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\web_video_4.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_4.mp4
hVideoSrc0 = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\optimized_prototype_1.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_4.mp4
hVideoSrc1 = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\optimized_prototype_2.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_4.mp4
hVideoSrc2 = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');

videoPlayer = vision.VideoPlayer;
opticFlow0 = opticalFlowLK();
opticFlow1 = opticalFlowLK();
opticFlow2 = opticalFlowLK();
frame_no=1;

count=1;
while ~isDone(hVideoSrc0)||~isDone(hVideoSrc1)||~isDone(hVideoSrc2)
    frame0 =(step(hVideoSrc0));
    crop_factor=100;
    frame0=frame0((crop_factor+1:size(frame0,1)-crop_factor),(crop_factor+1:size(frame0,2)-crop_factor));
    frame1 =(step(hVideoSrc1));
    frame2 =(step(hVideoSrc2));
    flow0 = estimateFlow(opticFlow0,frame0);
    flow1 = estimateFlow(opticFlow1,frame1);
    flow2 = estimateFlow(opticFlow2,frame2);
    f=figure(1);
    set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    plot(flow0,'DecimationFactor',[20 20],'ScaleFactor',30);
    hold on;
    plot(flow1,'DecimationFactor',[20 20],'ScaleFactor',30);
    hold on;
    plot(flow2,'DecimationFactor',[20 20],'ScaleFactor',30);
    hold off;
    set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    F = getframe(f);
    writeVideo(video_write,F.cdata); 
    if frame_no>=101&&frame_no<=104
        imwrite(uint8(F.cdata),strcat(num2str(count),'.jpg'));
        count=count+1;
    end
    frame_no=frame_no+1
end

close(video_write);
% Here you call the release method on the objects to close any open files
% and release memory.
release(hVideoSrc0);
release(hVideoSrc1);
release(hVideoSrc2);