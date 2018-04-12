clc;
clear;
close all;
v = VideoReader('simulated_vibration_video.mp4');
%Options-simulated_vibration_video,horizontal_pan_video
N=v.NumberOfFrames;

stabilized_video_AMV = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\stabilized_video_AMV.mp4','MPEG-4');
stabilized_video_AMV.FrameRate = 30;
open(stabilized_video_AMV);

comparison_video = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\comparison_video.mp4','MPEG-4');
comparison_video.FrameRate = 30;
open(comparison_video);

stabilized_video_CMV = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\stabilized_video_CMV.mp4','MPEG-4');
stabilized_video_CMV.FrameRate = 30;
open(stabilized_video_CMV);

stabilized_video_FMV = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\stabilized_video_FMV.mp4','MPEG-4');
stabilized_video_FMV.FrameRate = 30;
open(stabilized_video_FMV);

H = vision.TemplateMatcher;
H.ROIInputPort=1;

frame = rgb2gray(v.read(1));
stabilized_frame=frame((51:(size(frame,1)-50)),(51:(size(frame,2)-50)));
writeVideo(stabilized_video_AMV,stabilized_frame);
writeVideo(stabilized_video_CMV,stabilized_frame);
writeVideo(stabilized_video_FMV,stabilized_frame);





frame_no=2;
vibration_vector_array_AMV=zeros(99,2);
vibration_vector_array_CMV=zeros(99,2);
vibration_vector_array_FMV=zeros(99,2);
% prev_frame=rgb2gray(v.read(1));
global_motion_vector=[0 0];
accumulated_motion_vector=[0 0];
constant_motion_vector=[0 0];
constant_motion_vector_I=[0 0];
modified_compensation_vector=[0 0];
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
    if(sorted_index(1,2)==8)
        FB_location=[size(cropped_frame,1)-49,middle_index_2-24];
    end
    if(sorted_index(1,2)==9)
        FB_location=[size(cropped_frame,1)-49,size(cropped_frame,2)-49];
    end
    
    FB_location=FB_location+50;
    ROI=[FB_location(2)-7,FB_location(1)-7,64 64];
    [LOC] = step(H,prev_frame,FB,ROI);
    LOC=LOC-24;
    LOC=double([LOC(2),LOC(1)]);
    
    k=0.9;
    alpha=0.95;
    accumulated_motion_vector=round(k*accumulated_motion_vector+alpha*(FB_location-LOC)+(1-alpha)*global_motion_vector);

    %     if ~isequal((abs(accumulated_motion_vector)>50),[0 0])
%         if accumulated_motion_vector(1)>50
%             accumulated_motion_vector(1)=50;
%         end
%         if accumulated_motion_vector(1)<-50
%             accumulated_motion_vector(1)=-50;
%         end
%         if accumulated_motion_vector(2)>50
%             accumulated_motion_vector(2)=50;
%         end
%         if accumulated_motion_vector(2)<-50
%             accumulated_motion_vector(2)=-50;
%         end
%     end
    
    k=0.95;
    beta=0.015;
    constant_motion_vector=round(k*constant_motion_vector+(FB_location-LOC)-beta*constant_motion_vector_I);
    constant_motion_vector_I=constant_motion_vector_I+constant_motion_vector;    
    
    gamma=0.5;
    modified_compensation_vector=round(gamma*accumulated_motion_vector+(1-gamma)*constant_motion_vector);
        
    global_motion_vector=(FB_location-LOC);
    
    
    
    vibration_vector_array_AMV(frame_no,:)=accumulated_motion_vector;
    vibration_vector_array_CMV(frame_no,:)=constant_motion_vector;
    vibration_vector_array_FMV(frame_no,:)=modified_compensation_vector;
    
%     
    stabilized_frame_AMV=frame((51:(size(frame,1)-50))+accumulated_motion_vector(1),(51:(size(frame,2)-50))+accumulated_motion_vector(2));
    stabilized_frame_CMV=frame((51:(size(frame,1)-50))+constant_motion_vector(1),(51:(size(frame,2)-50))+constant_motion_vector(2));
    stabilized_frame_FMV=frame((51:(size(frame,1)-50))+modified_compensation_vector(1),(51:(size(frame,2)-50))+modified_compensation_vector(2));
    comparison_frame=frame((51:(size(frame,1)-50)),(51:(size(frame,2)-50)));
    writeVideo(stabilized_video_AMV,stabilized_frame_AMV);
    writeVideo(stabilized_video_CMV,stabilized_frame_CMV);
    writeVideo(comparison_video,comparison_frame);
    writeVideo(stabilized_video_FMV,stabilized_frame_FMV);
    frame_no=frame_no+1
end
close(stabilized_video_AMV);
close(comparison_video);
close(stabilized_video_CMV);
close(stabilized_video_FMV);

load('actual_vibration_vector.mat');
figure;
plot(vibration_vector_array(:,1),'r');
hold on;
plot(vibration_vector_array_AMV(:,1),'b');
figure;
plot(vibration_vector_array(:,2),'r');
hold on;
plot(vibration_vector_array_AMV(:,2),'b');

figure;
plot(vibration_vector_array(:,1),'r');
hold on;
plot(vibration_vector_array_CMV(:,1),'b');
figure;
plot(vibration_vector_array(:,2),'r');
hold on;
plot(vibration_vector_array_CMV(:,2),'b');

figure;
plot(vibration_vector_array(:,1),'r');
hold on;
plot(vibration_vector_array_FMV(:,1),'b');
figure;
plot(vibration_vector_array(:,2),'r');
hold on;
plot(vibration_vector_array_FMV(:,2),'b');
