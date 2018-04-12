clc;
clear;
close all;

v = VideoReader('horizontal_pan_video.mp4');
N=v.NumberOfFrames;



outputVideo = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\simulated_vibration_video_horizontal_pan.mp4','MPEG-4');
outputVideo.FrameRate = 30;
open(outputVideo);

frame = rgb2gray(v.read(1));
frame=frame(27:end-26,:);
frame1=frame((11:end-10),(11:end-10));
writeVideo(outputVideo,frame1);
vibration_vector_array=zeros(N-1,2);
frame_no=2;
while frame_no<=N
    frame = rgb2gray(v.read(frame_no));
    frame=frame(27:end-26,:);
    vibration_vector=[randi([-3 3]),randi([-3 3])];
    vibration_vector_array(frame_no,:)=vibration_vector;
    vibration_induced_frame=frame((11:end-10)+vibration_vector(1),(11:end-10)+vibration_vector(2));
    writeVideo(outputVideo,vibration_induced_frame);
    frame_no=frame_no+1
end


close(outputVideo);