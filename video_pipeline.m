clc;
clear;
close all;
%Main template taken from https://www.mathworks.com/help/vision/examples/face-detection-and-tracking-using-live-video-acquisition.html

%Initialization

H = vision.TemplateMatcher;
H.ROIInputPort=1;
mean=zeros(1,9);
variance=zeros(1,9);
constant_motion_vector=[0 0];
constant_motion_vector_I=[0 0];

cam = webcam();



% Create the video player object.
%videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

stabilized_video_webcam = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\stabilized_video_webcam.mp4','MPEG-4');
stabilized_video_webcam.FrameRate = 30;
open(stabilized_video_webcam);
frame = rgb2gray(snapshot(cam));
stabilized_frame=frame((51:(size(frame,1)-50)),(51:(size(frame,2)-50)));
writeVideo(stabilized_video_webcam,stabilized_frame);

frameCount = 0;

prev_frame = rgb2gray(snapshot(cam));
while frameCount < 90
    
    global_motion_vector=[ 0 0];
    frame = rgb2gray(snapshot(cam));
    frameCount = frameCount + 1
    
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
    FB=reshape(FOB_array(:,:,9),[50 50]);
    
    if(sorted_index(1,9)==1)
    FB_location=[1,1];
    end
    if(sorted_index(1,9)==2)
        FB_location=[1,middle_index_2-24];
    end
    if(sorted_index(1,9)==3)
        FB_location=[1,size(cropped_frame,2)-49];
    end
    if(sorted_index(1,9)==4)
        FB_location=[middle_index_1-24,1];
    end
    if(sorted_index(1,9)==5)
        FB_location=[middle_index_1-24,middle_index_2-24];
    end
    if(sorted_index(1,9)==6)
        FB_location=[middle_index_1-24,size(cropped_frame,2)-49];
    end
    if(sorted_index(1,9)==7)
        FB_location=[size(cropped_frame,1)-49,1];
    end
    if(sorted_index(1,9)==8)
        FB_location=[size(cropped_frame,1)-49,middle_index_2-24];
    end
    if(sorted_index(1,9)==9)
        FB_location=[size(cropped_frame,1)-49,size(cropped_frame,2)-49];
    end
    
    FB_location=FB_location+50;
    ROI=[FB_location(2)-7,FB_location(1)-7,64 64];
    [LOC] = step(H,prev_frame,FB,ROI);
    LOC=LOC-24;
    LOC=double([LOC(2),LOC(1)]);
    
    global_motion_vector=global_motion_vector+0.5*(FB_location-LOC);
    
    FB_2=reshape(FOB_array(:,:,8),[50 50]);
    
    if(sorted_index(1,8)==1)
        FB_2_location=[1,1];
    end
    if(sorted_index(1,8)==2)
        FB_2_location=[1,middle_index_2-24];
    end
    if(sorted_index(1,8)==3)
        FB_2_location=[1,size(cropped_frame,2)-49];
    end
    if(sorted_index(1,8)==4)
        FB_2_location=[middle_index_1-24,1];
    end
    if(sorted_index(1,8)==5)
        FB_2_location=[middle_index_1-24,middle_index_2-24];
    end
    if(sorted_index(1,8)==6)
        FB_2_location=[middle_index_1-24,size(cropped_frame,2)-49];
    end
    if(sorted_index(1,8)==7)
        FB_2_location=[size(cropped_frame,1)-49,1];
    end
    if(sorted_index(1,8)==8)
        FB_2_location=[size(cropped_frame,1)-49,middle_index_2-24];
    end
    if(sorted_index(1,8)==9)
        FB_2_location=[size(cropped_frame,1)-49,size(cropped_frame,2)-49];
    end
    
    FB_2_location=FB_2_location+50;
    ROI=[FB_2_location(2)-7,FB_2_location(1)-7,64 64];
    [LOC_2] = step(H,prev_frame,FB_2,ROI);
    LOC_2=LOC_2-24;
    LOC_2=double([LOC_2(2),LOC_2(1)]);
    
    global_motion_vector=global_motion_vector+0.5*(FB_2_location-LOC_2);
    
    k=0.9;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)
    beta=0.015;%0.015
    constant_motion_vector=round(k*constant_motion_vector+(global_motion_vector)-beta*constant_motion_vector_I);
    constant_motion_vector_I=constant_motion_vector_I+constant_motion_vector;
    
    
    stabilized_frame_webcam=frame((51:(size(frame,1)-50))+constant_motion_vector(1),(51:(size(frame,2)-50))+constant_motion_vector(2));

    

    % Display the annotated video frame using the video player object.
    
    writeVideo(stabilized_video_webcam,stabilized_frame_webcam);

    % Check whether the video player window has been closed.
    
    prev_frame=frame;
end

% Clean up.
clear cam;
close(stabilized_video_webcam);

