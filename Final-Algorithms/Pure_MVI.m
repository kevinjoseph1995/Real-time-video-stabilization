clc;
clear;
close all;
%***************************************************************************
%Video Write-to-file initialization
video_write_obj = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\pure_mvi.mp4','MPEG-4');
video_write_obj.FrameRate = 30;
open(video_write_obj);
%***************************************************************************
%Video Read-from-file Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\web_video_4.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_2.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
%***************************************************************************
%Other Initializations
H = vision.TemplateMatcher;
H.ROIInputPort=1;
counts=zeros(9,256);
mean=zeros(1,9);
variance=zeros(1,9);
constant_motion_vector=[0 0];
constant_motion_vector_I=[0 0];

%********************************
frame_no=1;
prev_frame=step(hVideoSrc);
prev_frame=padarray(prev_frame,[60 60],0);
while ~isDone(hVideoSrc)
    %Re-initializing global motion vector
    global_motion_vector=[ 0 0];
    %Reading current frame
    frame=step(hVideoSrc);
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
    ROI=[FB_location_temp(2)-10,FB_location_temp(1)-10,70 70];
    [LOC] = step(H,prev_frame,FB,ROI);
    LOC=LOC-24;
    LOC=double([LOC(2),LOC(1)]);    
    global_motion_vector=global_motion_vector+1*(FB_location_temp-LOC);
    
    %************************************************************************
    %Correction
    
    k=0.95 ;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)
    beta=0.015;%0.015
    constant_motion_vector=round(k*constant_motion_vector+(global_motion_vector)-beta*constant_motion_vector_I);
    constant_motion_vector_I=constant_motion_vector_I+constant_motion_vector;
    
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
    
    stabilized_frame=frame((111:(size(frame,1)-110))+constant_motion_vector(1),(111:(size(frame,2)-110))+constant_motion_vector(2));
    writeVideo(video_write_obj,stabilized_frame);    
    %Resetting previous frame with current frame, as we move onto the next iteration
    prev_frame=frame;
    frame_no=frame_no+1
end
close(video_write_obj);
release(hVideoSrc);