clc;
clear;
close all;
%***************************************************************************
%Video Read Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\pure_mvi.mp4';
%filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\simulated_vibration_video_horizontal_pan.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_2.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
%************
H = vision.TemplateMatcher;
H.ROIInputPort=1;
trajectory_vector_accumulator=[0 0];
trajectory_vector_accumulator_affine=[0 0];
%************
frame_no=1;
prev_frame=step(hVideoSrc);
while ~isDone(hVideoSrc)
    %Re-initializing global motion vector
    global_motion_vector=[ 0 0];
    current_frame=step(hVideoSrc);    
    frame=padarray(current_frame,[60 60],0);
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
    global_motion_vector=global_motion_vector+(FB_location_temp-LOC);
    
      
    trajectory_vector_accumulator=trajectory_vector_accumulator+global_motion_vector;
    trajectory_no_compensation_hist_method(frame_no,:)=trajectory_vector_accumulator;
    frame_no=frame_no+1
end
figure(1);
plot(trajectory_no_compensation_hist_method(:,1));
title('X Displacement');
xlabel('Frame Number');
ylabel('Displacement');
hold on;
figure(2);
plot(trajectory_no_compensation_hist_method(:,2));
title('Y Displacement');
xlabel('Frame Number');
ylabel('Displacement');
hold on;
%**************************************************************
clear;

%***************************************************************************
%Video Read Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\mvi_affine.mp4';
%filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\simulated_vibration_video_horizontal_pan.mp4';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_2.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
%************
H = vision.TemplateMatcher;
H.ROIInputPort=1;
trajectory_vector_accumulator=[0 0];
trajectory_vector_accumulator_affine=[0 0];
%************
frame_no=1;
prev_frame=step(hVideoSrc);
while ~isDone(hVideoSrc)
    %Re-initializing global motion vector
    global_motion_vector=[ 0 0];
    current_frame=step(hVideoSrc);    
    frame=padarray(current_frame,[60 60],0);
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
    global_motion_vector=global_motion_vector+(FB_location_temp-LOC);
    
      
    trajectory_vector_accumulator=trajectory_vector_accumulator+global_motion_vector;
    trajectory_no_compensation_hist_method(frame_no,:)=trajectory_vector_accumulator;
    frame_no=frame_no+1
end
figure(1);
plot(trajectory_no_compensation_hist_method(:,1));
title('X Displacement');
xlabel('Frame Number');
ylabel('Displacement');
hold on;
figure(2);
plot(trajectory_no_compensation_hist_method(:,2));
title('Y Displacement');
xlabel('Frame Number');
ylabel('Displacement');
hold on;

