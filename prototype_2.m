clc;
clear;
close all;
v = VideoReader('simulated_vibration_video_horizontal_pan.mp4');

%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan

%Initialization
N=v.NumberOfFrames;
H = vision.TemplateMatcher;
H.ROIInputPort=1;
trajectory_vector_accumulator=[0 0];
trajectory_no_compensation=zeros(N-1,2);
trajectory_with_CMVcompensation=zeros(N-1,2);
constant_motion_vector=[0 0];
constant_motion_vector_I=[0 0];

%Video Initialization

stabilized_video_prototype_2 = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\stabilized_video_prototype_2.mp4','MPEG-4');
stabilized_video_prototype_2.FrameRate = 30;
open(stabilized_video_prototype_2);
frame = rgb2gray(v.read(1));
stabilized_frame=frame((51:(size(frame,1)-50)),(51:(size(frame,2)-50)));
writeVideo(stabilized_video_prototype_2,stabilized_frame);
frame_no=2;
prev_frame=rgb2gray(v.read(1));
stabilized_frame=prev_frame((51:(size(prev_frame,1)-50)),(51:(size(prev_frame,2)-50)));
while frame_no<=N
    global_motion_vector=[ 0 0];
    frame = rgb2gray(v.read(frame_no));
    cropped_frame=frame(51:(size(frame,1)-50),51:(size(frame,2)-50));
    FB_location=zeros(10,2);
    FOB_array=zeros(50,50,10);
    for i=1:1  
        %choose random patch within current frame
        FB_location(i,:)=[randi([1 size(cropped_frame,1)-49],1), randi([1 size(cropped_frame,2)-49],1)];
        FOB_array(:,:,i)=cropped_frame(FB_location(i,1):FB_location(i,1)+49,FB_location(i,2):FB_location(i,2)+49);
    
        FB_location(i,:)=FB_location(i,:)+50;
        global_motion_vector=[0 0];

        template=(FOB_array(:,:,i));
        image=(prev_frame(FB_location(i,1)-10:FB_location(i,1)+49+10,FB_location(i,2)-10:FB_location(i,2)+49+10));
        c=normxcorr2(template,image);
        [ypeak, xpeak] = find(c==max(c(:)));
        yoffSet = (ypeak-size(FOB_array(:,:,i),1))+1;
        xoffSet = (xpeak-size(FOB_array(:,:,i),2))+1;
        %locate the position of the random patch in the previous frame
        LOC=[yoffSet+FB_location(i,1)-11,xoffSet+FB_location(i,2)-11];
        %Calculation of  motion vector's for particular patch and accumulating it.
        global_motion_vector=global_motion_vector+(FB_location(i,:)-LOC);
        
    end
    %averaging all the motion vector's to find the global motion vector. 
    global_motion_vector=global_motion_vector/3;
    
    k=0.99;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)
    beta=0.015;%0.015
    constant_motion_vector=round(k*constant_motion_vector+(global_motion_vector)-beta*constant_motion_vector_I);
    constant_motion_vector_I=constant_motion_vector_I+constant_motion_vector;
    
    trajectory_vector_accumulator=trajectory_vector_accumulator+global_motion_vector;
    trajectory_no_compensation(frame_no-1,:)=trajectory_vector_accumulator;
    trajectory_with_CMVcompensation(frame_no-1,:)=trajectory_vector_accumulator-constant_motion_vector;
    
    stabilized_frame=frame((51:(size(frame,1)-50))+constant_motion_vector(1),(51:(size(frame,2)-50))+constant_motion_vector(2));
    writeVideo(stabilized_video_prototype_2,stabilized_frame);
    prev_frame=frame;
    frame_no=frame_no+1;
end
close(stabilized_video_prototype_2);
plot(trajectory_no_compensation(:,1),'r');
hold on;
plot(trajectory_no_compensation(:,2),'b');
hold on;
plot(trajectory_with_CMVcompensation(:,1),'r');
hold on;
plot(trajectory_with_CMVcompensation(:,2),'b');

