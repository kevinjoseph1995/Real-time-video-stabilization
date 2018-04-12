clc;
clear;
close all;
image=rgb2gray(imread('test_image.png'));

outputVideo = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\simulated_vibration_video.mp4','MPEG-4');
outputVideo.FrameRate = 30;
open(outputVideo);

image_cropped=image((51:650),(51:650));
writeVideo(outputVideo,image_cropped)

vibration_vector_array=zeros(99,2);

for frame_no=2:100
    vibration_vector=[randi([-3 3]),randi([-3 3])];
    vibration_vector_array(frame_no,:)=vibration_vector;
    image_cropped=image((51:650)+vibration_vector(1),(51:650)+vibration_vector(2));
    writeVideo(outputVideo,image_cropped)
end
close(outputVideo);