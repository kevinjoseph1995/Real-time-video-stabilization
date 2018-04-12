clc;
clear;
close all;
v = VideoReader('simulated_vibration_video_horizontal_pan.mp4');
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan

N=v.NumberOfFrames;

stabilized_video_CMV = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\stabilized_video_CMV.mp4','MPEG-4');
stabilized_video_CMV.FrameRate = 30;
open(stabilized_video_CMV);

H = vision.TemplateMatcher;
H.ROIInputPort=1;

frame = rgb2gray(v.read(1));
stabilized_frame=frame((51:(size(frame,1)-50)),(51:(size(frame,2)-50)));
writeVideo(stabilized_video_CMV,stabilized_frame);


frame_no=2;
global_motion_vector=[ 0 0];
accumulated_motion_vector=[0 0];
constant_motion_vector=[0 0];
constant_motion_vector_I=[0 0];
modified_compensation_vector=[0 0];

trajectory_vector_accumulator=[0 0];

trajectory_with_AMVcompensation=zeros(N-1,2);
trajectory_with_CMVcompensation=zeros(N-1,2);
trajectory_with_FMVcompensation=zeros(N-1,2);
trajectory_no_compensation=zeros(N-1,2);
while frame_no<=N
    frame = rgb2gray(v.read(frame_no));
    prev_frame=rgb2gray(v.read(frame_no-1));
    cropped_frame=frame(51:(size(frame,1)-50),51:(size(frame,2)-50));
    
    middle_index_1=round(size(cropped_frame,1)/2);
    middle_index_2=round(size(cropped_frame,2)/2);    
    
    FOB_array(:,:,1)=cropped_frame(1:50,1:50);
    FOB_array(:,:,2)=cropped_frame(1:50,middle_index_2-24:middle_index_2+25);
    FOB_array(:,:,3)=cropped_frame(1:50,size(cropped_frame,2)-49:size(cropped_frame,2));
    
    FOB_array(:,:,4)=cropped_frame(middle_index_1-24:middle_index_1+25,1:50);
    FOB_array(:,:,5)=cropped_frame(middle_index_1-24:middle_index_1+25,middle_index_2-24:middle_index_2+25);
    FOB_array(:,:,6)=cropped_frame(middle_index_1-24:middle_index_1+25,size(cropped_frame,2)-49:size(cropped_frame,2));
    
    FOB_array(:,:,7)=cropped_frame(size(cropped_frame,1)-49:size(cropped_frame,1),1:50);
    FOB_array(:,:,8)=cropped_frame(size(cropped_frame,1)-49:size(cropped_frame,1),middle_index_2-24:middle_index_2+25);
    FOB_array(:,:,9)=cropped_frame(size(cropped_frame,1)-49:size(cropped_frame,1),size(cropped_frame,2)-49:size(cropped_frame,2));
    
    [counts(1,:),binLocations] = imhist(FOB_array(:,:,1));
    [counts(2,:),binLocations] = imhist(FOB_array(:,:,2));
    [counts(3,:),binLocations] = imhist(FOB_array(:,:,3));
    [counts(4,:),binLocations] = imhist(FOB_array(:,:,4));
    [counts(5,:),binLocations] = imhist(FOB_array(:,:,5));
    [counts(6,:),binLocations] = imhist(FOB_array(:,:,6));
    [counts(7,:),binLocations] = imhist(FOB_array(:,:,7));
    [counts(8,:),binLocations] = imhist(FOB_array(:,:,8));
    [counts(9,:),binLocations] = imhist(FOB_array(:,:,9));
    
    c=1;
    for i=1:5:255
        count(:,c)=zeros(9,1);
        for j=i:i+4
            count(:,c)=count(:,c)+counts(:,j);
        end
        c=c+1;
    end
    count=count';
    clear counts;
    [sorted_count,sorted_index]=sort(max(count));
    clear count;
    FOB_array=FOB_array(:,:,sorted_index);
    
    FB=reshape(FOB_array(:,:,1),[50 50]);
    
    if(sorted_index(1,1)==1)
    FB_location=[1,1];
    end
    if(sorted_index(1,1)==2)
        FB_location=[1,middle_index_2-24];
    end
    if(sorted_index(1,1)==3)
        FB_location=[1,size(cropped_frame,2)-49];
    end
    if(sorted_index(1,1)==4)
        FB_location=[middle_index_1-24,1];
    end
    if(sorted_index(1,1)==5)
        FB_location=[middle_index_1-24,middle_index_2-24];
    end
    if(sorted_index(1,1)==6)
        FB_location=[middle_index_1-24,size(cropped_frame,2)-49];
    end
    if(sorted_index(1,1)==7)
        FB_location=[size(cropped_frame,1)-49,1];
    end
    if(sorted_index(1,1)==8)
        FB_location=[size(cropped_frame,1)-49,middle_index_2-24];
    end
    if(sorted_index(1,2)==9)
        FB_location=[size(cropped_frame,1)-49,size(cropped_frame,2)-49];
    end
    
    FB_location=FB_location+50;
    ROI=[FB_location(2)-6,FB_location(1)-6,62 62];
    [LOC] = step(H,prev_frame,FB,ROI);
    LOC=LOC-24;
    LOC=double([LOC(2),LOC(1)]);
    
    
    k=0.9;
    alpha=0.95;
    accumulated_motion_vector=round(k*accumulated_motion_vector+alpha*(FB_location-LOC)+(1-alpha)*global_motion_vector);
    
    k=0.1;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)
    beta=0.015;%0.015
    constant_motion_vector=round(k*constant_motion_vector+(FB_location-LOC)-beta*constant_motion_vector_I);
    constant_motion_vector_I=constant_motion_vector_I+constant_motion_vector;    
    
    gamma=0.5;
    modified_compensation_vector=round(gamma*accumulated_motion_vector+(1-gamma)*constant_motion_vector);
    
    global_motion_vector=(FB_location-LOC);
    
    
    stabilized_frame_CMV=frame((51:(size(frame,1)-50))+constant_motion_vector(1),(51:(size(frame,2)-50))+constant_motion_vector(2));
    
    writeVideo(stabilized_video_CMV,stabilized_frame_CMV);    
    
    
    trajectory_vector_accumulator=trajectory_vector_accumulator+global_motion_vector;
    trajectory_no_compensation(frame_no-1,:)=trajectory_vector_accumulator;
    trajectory_with_AMVcompensation(frame_no-1,:)=trajectory_vector_accumulator-accumulated_motion_vector;
    trajectory_with_CMVcompensation(frame_no-1,:)=trajectory_vector_accumulator-constant_motion_vector;
    trajectory_with_FMVcompensation(frame_no-1,:)=trajectory_vector_accumulator-modified_compensation_vector;
    
    frame_no=frame_no+1
end
close(stabilized_video_CMV);
    

plot(trajectory_no_compensation(:,1),'r');
hold on;
plot(trajectory_with_AMVcompensation(:,1),'b');
plot(trajectory_with_CMVcompensation(:,1),'g');
plot(trajectory_with_FMVcompensation(:,1),'y');
figure;
plot(trajectory_no_compensation(:,2));
hold on;
plot(trajectory_with_AMVcompensation(:,2));
plot(trajectory_with_CMVcompensation(:,2));
plot(trajectory_with_FMVcompensation(:,2));