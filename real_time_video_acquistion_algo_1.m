clc;
clear;
close all;
%Kevin Joseph 13BEC0063 

% Initializations

% 1 Video Player Initialization
videoPlayer = vision.VideoPlayer;
% 2 Image Acquistion Initialization
vid = videoinput('winvideo',2,'RGB24_640x480');
vid.TriggerRepeat = Inf;
vid.FramesPerTrigger=1;
% 3 Other Initialzation
FMV=[0 0];
constant_motion_vector_I=[0 0];
global_motion_vector=[0 0];
H = vision.TemplateMatcher;
H.ROIInputPort=1;
counts=(zeros(3,256));
avg=(zeros(1,3));
variance=(zeros(1,3));
% Start acquiring frames.
start(vid)
loopCounter=1;
while(vid.FramesAvailable >= 0)
    data = getdata(vid,2);      
    prev_frame=rgb2gray(data(:,:,:,2));
    current_frame=rgb2gray(data(:,:,:,1));     
    %Cropping Current Frame
    frame_crop_size=5;
    FOB_size=30;%TWEAKABLE PARAMETER too small and you incur huge motion estimation loss; too large and frame rate drops
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
    ROI_width=1;%TWEAKABLE PARAMETER
    ROI=[FB_location_temp(2)-ROI_width,FB_location_temp(1)-ROI_width,2*ROI_width+FOB_size,2*ROI_width+FOB_size];
    [LOC] = step(H,(prev_frame),(FB),(ROI));
    LOC=LOC-(round(FOB_size/2)-1);
    LOC=double([LOC(2),LOC(1)]);    
    global_motion_vector_prev=global_motion_vector;
    global_motion_vector=(FB_location_temp-LOC);
    %Correction    
    k=0.99 ;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)%TWEAKABLE PARAMETER
    beta=0.015;%0.015%TWEAKABLE PARAMETER
    alpha=0;%TWEAKABLE PARAMETER (between 0 and 1)
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
%     stabilized_frame=imsharpen(stabilized_frame,'Amount',1.5);
    temp_frame= mat2gray(current_frame((frame_crop_size+1:(size(current_frame,1)-frame_crop_size)),(frame_crop_size+1:(size(current_frame,2)-frame_crop_size)))); 
    step(videoPlayer, ([temp_frame stabilized_frame]));
    loopCounter=loopCounter+1;
    

end
