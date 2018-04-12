clc;
clear;
close all;
v = VideoReader('web_video_4.mp4');
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov

%Initialization
N=v.NumberOfFrames;
H = vision.TemplateMatcher;
H.ROIInputPort=1;
counts=zeros(9,256);
mean=zeros(1,9);
variance=zeros(1,9);
trajectory_vector_accumulator=[0 0];
trajectory_no_compensation=zeros(N-1,2);
constant_motion_vector=[0 0];
constant_motion_vector_I=[0 0];

stabilized_video_prototype_3 = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\stabilized_video_prototype_3_temp1.mp4','MPEG-4');
stabilized_video_prototype_3.FrameRate = 30;
open(stabilized_video_prototype_3);
frame = rgb2gray(v.read(1));
frame=padarray(frame,[60 60],0);
stabilized_frame=frame((111:(size(frame,1)-110)),(111:(size(frame,2)-110)));
stabilized_frame=[stabilized_frame stabilized_frame];
stabilized_frame=imresize(stabilized_frame,0.813);%0.813% 0.2565 this scaling factor is specific to video format
writeVideo(stabilized_video_prototype_3,stabilized_frame);

frame_no=2;
%Reading First Frame
prev_frame=rgb2gray(v.read(1));
frame=padarray(prev_frame,[60 60],0);
%Begining
while frame_no<=N
    %Re-initializing global motion vector
    global_motion_vector=[ 0 0];
    %Reading subsequent frames
    frame = rgb2gray(v.read(frame_no));
    frame=padarray(frame,[60 60],0);
    %Cropping Current Frame
    cropped_frame=frame(111:(size(frame,1)-110),111:(size(frame,2)-110));
    
    middle_index_1=round(size(cropped_frame,1)/2);
    middle_index_2=round(size(cropped_frame,2)/2);    
    
    %Locating and storing the observation Blocks
    FOB_array(:,:,1)=cropped_frame(1:50,1:50);
    FOB_array(:,:,2)=cropped_frame(1:50,middle_index_2-24:middle_index_2+25);
    FOB_array(:,:,3)=cropped_frame(1:50,size(cropped_frame,2)-49:size(cropped_frame,2));    
    FOB_array(:,:,4)=cropped_frame(middle_index_1-24:middle_index_1+25,1:50);
    FOB_array(:,:,5)=cropped_frame(middle_index_1-24:middle_index_1+25,middle_index_2-24:middle_index_2+25);
    FOB_array(:,:,6)=cropped_frame(middle_index_1-24:middle_index_1+25,size(cropped_frame,2)-49:size(cropped_frame,2));    
    FOB_array(:,:,7)=cropped_frame(size(cropped_frame,1)-49:size(cropped_frame,1),1:50);
    FOB_array(:,:,8)=cropped_frame(size(cropped_frame,1)-49:size(cropped_frame,1),middle_index_2-24:middle_index_2+25);
    FOB_array(:,:,9)=cropped_frame(size(cropped_frame,1)-49:size(cropped_frame,1),size(cropped_frame,2)-49:size(cropped_frame,2));
    
    for i=1:9
        [counts(i,:),~] = imhist(FOB_array(:,:,i));
    end
    FB_location=[1,1;
                 1,middle_index_2-24
                 1,size(cropped_frame,2)-49
                 middle_index_1-24,1
                 middle_index_1-24,middle_index_2-24
                 middle_index_1-24,size(cropped_frame,2)-49
                 size(cropped_frame,1)-49,1
                 size(cropped_frame,1)-49,middle_index_2-24
                 size(cropped_frame,1)-49,size(cropped_frame,2)-49];
     %Sorting the observation blocks based on histogram spread        
     for i=1:9
        temp=0;
        for j=0:255
            temp=temp+j*counts(i,j+1);
        end
        mean(i)=round(temp/sum(counts(i,:)));
        temp=0;
        for j=0:255
            temp=temp+(counts(i,j+1)*(j-mean(i))^2);
        end
        variance(i)=round(temp/sum(counts(i,:)));
    end
    [sorted_count,sorted_index]=sort(variance);
    FOB_array=FOB_array(:,:,sorted_index);
    FB_location=FB_location(sorted_index,:);
    %First Feature Block
    FB=reshape(FOB_array(:,:,9),[50 50]);
    FB_location_temp=FB_location(9,:)+110;
    ROI=[FB_location_temp(2)-7,FB_location_temp(1)-7,64 64];
    [LOC] = step(H,prev_frame,FB,ROI);
    LOC=LOC-24;
    LOC=double([LOC(2),LOC(1)]);    
    global_motion_vector=global_motion_vector+1*(FB_location_temp-LOC);
%     Second Feature Block
%     FB=reshape(FOB_array(:,:,8),[50 50]);
%     FB_location_temp=FB_location(8,:)+50;
%     ROI=[FB_location_temp(2)-7,FB_location_temp(1)-7,64 64];
%     [LOC] = step(H,prev_frame,FB,ROI);
%     LOC=LOC-24;
%     LOC=double([LOC(2),LOC(1)]);    
%     global_motion_vector=global_motion_vector+0.5*(FB_location_temp-LOC);
    

    k=0.95 ;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)
    beta=0.015;%0.015
    constant_motion_vector=round(k*constant_motion_vector+(global_motion_vector)-beta*constant_motion_vector_I);
    constant_motion_vector_I=constant_motion_vector_I+constant_motion_vector;   
    
    
    
    trajectory_vector_accumulator=trajectory_vector_accumulator+global_motion_vector;
    trajectory_no_compensation(frame_no-1,:)=trajectory_vector_accumulator;
    
    if constant_motion_vector(1)>110
        constant_motion_vector(1)=110;
    end
    if constant_motion_vector(1)<-110
        constant_motion_vector(1)=-110;
    end
    if constant_motion_vector(2)>110
        constant_motion_vector(2)=110;
    end
    if constant_motion_vector(2)<-110
        constant_motion_vector(2)=-110;
    end
    
    temp_frame=frame((111:(size(frame,1)-110)),(111:(size(frame,2)-110)));
    stabilized_frame=frame((111:(size(frame,1)-110))+constant_motion_vector(1),(111:(size(frame,2)-110))+constant_motion_vector(2));
    new_frame=[temp_frame stabilized_frame];
    new_frame=imresize(new_frame,0.813);%0.813, 0.2565
    writeVideo(stabilized_video_prototype_3,new_frame);
    
    
    %Resetting previous frame with current frame, as we move onto the next iteration
    prev_frame=frame;
    frame_no=frame_no+1   
end
close(stabilized_video_prototype_3);
plot(trajectory_no_compensation(:,1),'b');
hold on;
plot(trajectory_no_compensation(:,2),'r');