clc;
clear;
close all;
%***************************************************************************
%Video Write-to-file initialization
stabilized_video_prototype = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\Final Algorithms\robotic_arm_stabilized.mp4','MPEG-4');
stabilized_video_prototype.FrameRate = 30;%TWEAKABLE PARAMETER
open(stabilized_video_prototype);

%Initialization
FMV=[0 0];
constant_motion_vector_I=[0 0];
Hcumulative = (eye(3));
global_motion_vector=[0 0];
H = vision.TemplateMatcher;
H.ROIInputPort=1;
counts=(zeros(3,256));
avg=(zeros(1,3));
variance=(zeros(1,3));

%Video Read Initialization
filename = 'C:\Users\Kevin Joseph\Desktop\Video Stabilization\robotic_arm_video\shake\2.avi';
%robotic_arm_video\shake\1.avi';
%Options-simulated_vibration_video,horizontal_pan_video,simulated_vibration_video_horizontal_pan,web_video_1.mov,web_video_2.mp4
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');

prev_frame=(step(hVideoSrc));
frame_no=1;
global_motion_vector=[0 0];

while ~isDone(hVideoSrc) 
    %Reading current frame
    current_frame=(step(hVideoSrc));   
    frame_crop_size=50;
    FOB_size=50;%TWEAKABLE PARAMETER too small and you incur huge motion estimation loss; too large and frame rate drops
    temp=(FOB_size+frame_crop_size);
    cropped_frame=current_frame(temp+1:(size(current_frame,1)-temp),temp+1:(size(current_frame,2)-temp));
    middle_index_1=round(size(cropped_frame,1)/2);
    middle_index_2=round(size(cropped_frame,2)/2);    
    %Locating and storing the observation Blocks
    FOB_array(:,:,1)=cropped_frame(1:FOB_size,1:FOB_size);
%     FOB_array(:,:,2)=cropped_frame(1:FOB_size,middle_index_2-(round(FOB_size/2)-1):middle_index_2+round(FOB_size/2));
%     FOB_array(:,:,3)=cropped_frame(1:FOB_size,size(cropped_frame,2)-(FOB_size-1):size(cropped_frame,2));    
%     FOB_array(:,:,4)=cropped_frame(middle_index_1-(round(FOB_size/2)-1):middle_index_1+round(FOB_size/2),1:FOB_size);
    FOB_array(:,:,2)=cropped_frame(middle_index_1-(round(FOB_size/2)-1):middle_index_1+round(FOB_size/2),middle_index_2-(round(FOB_size/2)-1):middle_index_2+round(FOB_size/2));
%     FOB_array(:,:,6)=cropped_frame(middle_index_1-(round(FOB_size/2)-1):middle_index_1+round(FOB_size/2),size(cropped_frame,2)-(FOB_size-1):size(cropped_frame,2));    
%     FOB_array(:,:,7)=cropped_frame(size(cropped_frame,1)-(FOB_size-1):size(cropped_frame,1),1:FOB_size);
%     FOB_array(:,:,8)=cropped_frame(size(cropped_frame,1)-(FOB_size-1):size(cropped_frame,1),middle_index_2-(round(FOB_size/2)-1):middle_index_2+round(FOB_size/2));
    FOB_array(:,:,3)=cropped_frame(size(cropped_frame,1)-(FOB_size-1):size(cropped_frame,1),size(cropped_frame,2)-(FOB_size-1):size(cropped_frame,2));
    FB_location=[1,1;
%                  1,middle_index_2-(round(FOB_size/2)-1)
%                  1,size(cropped_frame,2)-(FOB_size-1)
%                  middle_index_1-(round(FOB_size/2)-1),1
                 middle_index_1-(round(FOB_size/2)-1),middle_index_2-(round(FOB_size/2)-1)
%                  middle_index_1-(round(FOB_size/2)-1),size(cropped_frame,2)-(FOB_size-1)
%                  size(cropped_frame,1)-(FOB_size-1),1
%                  size(cropped_frame,1)-(FOB_size-1),middle_index_2-(round(FOB_size/2)-1)
                 size(cropped_frame,1)-(FOB_size-1),size(cropped_frame,2)-(FOB_size-1)];

    for i=1:3
        [counts(i,:),~] = imhist((FOB_array(:,:,i)));
    end

    %Sorting the observation blocks based on histogram spread 
    for i=1:3
        temp=0;
        for j=0:255
            temp=temp+j*counts(i,j+1);
        end
        avg(i)=round(temp/sum(counts(i,:)));
        temp=0;
        for j=0:255
            temp=temp+(counts(i,j+1)*(j-avg(i))^2);
        end
        variance(i)=round(temp/sum(counts(i,:)));
    end
    [sorted_count,sorted_index]=sort(variance);
    FOB_array=FOB_array(:,:,sorted_index);
    FB_location=FB_location(sorted_index,:);
    %First Feature Block
    FB=reshape(FOB_array(:,:,3),[FOB_size FOB_size]);
    temp=(FOB_size+frame_crop_size);
    FB_location_temp=FB_location(3,:)+temp;
    ROI_width=5;%TWEAKABLE PARAMETER
    ROI=[FB_location_temp(2)-ROI_width,FB_location_temp(1)-ROI_width,2*ROI_width+FOB_size,2*ROI_width+FOB_size];
    [LOC] = step(H,(prev_frame),(FB),(ROI));
    LOC=LOC-(round(FOB_size/2)-1);
    LOC=double([LOC(2),LOC(1)]);    
    global_motion_vector_prev=global_motion_vector;
    global_motion_vector=(FB_location_temp-LOC);
    %Correction    
    k=0.95 ;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)%TWEAKABLE PARAMETER
    beta=0.015;%0.015%TWEAKABLE PARAMETER
    alpha=1;%TWEAKABLE PARAMETER (between 0 and 1)
    gamma=1;%TWEAKABLE PARAMETER (between 0 and 1)
    FMV=round(k*FMV+...
    (gamma*(alpha*global_motion_vector+(1-alpha)*global_motion_vector_prev))+...
    ((1-gamma)*(global_motion_vector-beta*constant_motion_vector_I)));
    constant_motion_vector_I=constant_motion_vector_I+FMV;    
    frame_crop_size=50;
    if FMV(1)>frame_crop_size
        FMV(1)=frame_crop_size;
    end
    if FMV(1)<-frame_crop_size
        FMV(1)=-frame_crop_size;
    end
    if FMV(2)>frame_crop_size
        FMV(2)=frame_crop_size;
    end
    if FMV(2)<-frame_crop_size
        FMV(2)=-frame_crop_size;
    end
    frame_crop_size=50;
    stabilized_frame=mat2gray((current_frame((frame_crop_size+1:(size(current_frame,1)-frame_crop_size))+FMV(1),...
    (frame_crop_size+1:(size(current_frame,2)-frame_crop_size))+FMV(2))));
    temp_frame= double(current_frame((frame_crop_size+1:(size(current_frame,1)-frame_crop_size)),(frame_crop_size+1:(size(current_frame,2)-frame_crop_size))));
    combined_frame=[temp_frame stabilized_frame];
    writeVideo(stabilized_video_prototype,(combined_frame));
    prev_frame=current_frame;
    frame_no=frame_no+1
    
end
close(stabilized_video_prototype);
% Here you call the release method on the objects to close any open files
% and release memory.
release(hVideoSrc);
%release(videoPlayer);
