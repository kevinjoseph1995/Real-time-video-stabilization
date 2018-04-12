clc;
clear;
close all;
%***************************************************************************
%Video Write-to-file initialization
video_write = VideoWriter('C:\Users\admin\Desktop\Video Stabilization\Final Algorithms\vibration_visualized_original.mp4','MPEG-4');
video_write.FrameRate = 30;%TWEAKABLE PARAMETER
open(video_write);


%Video Read Initialization
filename = 'C:\Users\admin\Desktop\Video Stabilization\web_video_4.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_2.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
frame_no=1;
opticFlow = opticalFlowLK('NoiseThreshold',0.009);
I=zeros(100,100,3,16);
count=1;
while ~isDone(hVideoSrc) 
    %Reading current frame
    current_frame=(step(hVideoSrc));
    frame_crop_size=100;
    current_frame=current_frame((frame_crop_size+1:(size(current_frame,1)-frame_crop_size)),(frame_crop_size+1:(size(current_frame,2)-frame_crop_size)));
    
    flow = estimateFlow(opticFlow,current_frame);
    f=figure(1);
    imshow(current_frame)
    hold on
    plot(flow,'DecimationFactor',[5 5],'ScaleFactor',10)
    hold off
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
    F=getframe(f);
    image=F.cdata;
    writeVideo(video_write,image);
    if frame_no>=101 && frame_no<=116
        I(:,:,:,count)=(image(201:300,501:600,:));
        count=count+1;
    end
    frame_no=frame_no+1;
end
I=uint8(I);
figure(2);
montage(I);
close(video_write);