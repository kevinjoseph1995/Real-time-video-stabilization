clc;
clear;
close all;
load('My Movie_4.mat');
H = vision.TemplateMatcher;

frame=MyMovie_4(:,:,:,1);
frame_gray=rgb2gray(frame);
frame_gray=frame_gray(:,110:745);
imwrite(frame_gray(51:430,51:586),'C:\Users\Kevin Joseph\Desktop\Video Stabilization\image_bin/1.jpg');

for frame_no=2:size(MyMovie_4,4)
    frame1=MyMovie_4(:,:,:,frame_no-1);
    frame2=MyMovie_4(:,:,:,frame_no);
    frame1_gray=rgb2gray(frame1);
    frame1_gray=frame1_gray(:,110:745);
    frame2_gray=rgb2gray(frame2);
    frame2_gray=frame2_gray(:,110:745);
    
    FOB_array(:,:,1)=frame2_gray(51:100,51:100);
    FOB_array(:,:,2)=frame2_gray(51:100,293:342);
    FOB_array(:,:,3)=frame2_gray(51:100,537:586);
    FOB_array(:,:,4)=frame2_gray(215:264,51:100);
    FOB_array(:,:,5)=frame2_gray(215:264,293:342);
    FOB_array(:,:,6)=frame2_gray(215:264,537:586);
    FOB_array(:,:,7)=frame2_gray(381:430,51:100);
    FOB_array(:,:,8)=frame2_gray(381:430,293:342);
    FOB_array(:,:,9)=frame2_gray(381:430,537:586);
    
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
    
    %checking if movement is there
    
    %Section incomplete
    
    %*****************************

    FB=reshape(FOB_array(:,:,1),[50 50]);
    if(sorted_index(1,1)==1)
    FB_location=[51,51];
    end
    if(sorted_index(1,1)==2)
        FB_location=[51,293];
    end
    if(sorted_index(1,1)==3)
        FB_location=[51,537];
    end
    if(sorted_index(1,1)==4)
        FB_location=[215,51];
    end
    if(sorted_index(1,1)==5)
        FB_location=[215,293];
    end
    if(sorted_index(1,1)==6)
        FB_location=[215,537];
    end
    if(sorted_index(1,1)==7)
        FB_location=[381,51];
    end
    if(sorted_index(1,1)==8)
        FB_location=[381,293];
    end
    if(sorted_index(1,1)==9)
        FB_location=[381,537];
    end
    
    LOC = step(H,frame1_gray,FB);
    LOC=LOC-24;
    LOC=double([LOC(2),LOC(1)]);
    vibration_vector=FB_location-LOC;
    vibration_vector=round(0.1*vibration_vector);

    restored_frame=frame2_gray((51:430)-vibration_vector(1),(51:586)-vibration_vector(2));
    str=strcat('C:\Users\Kevin Joseph\Desktop\Video Stabilization\image_bin\',num2str(frame_no),'.jpg');
    imwrite(restored_frame,str);
end