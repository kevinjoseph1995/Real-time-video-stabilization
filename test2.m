clc;
clear;
close all;
load('successive_frames_video4.mat');
frame1_gray=rgb2gray(frame1);
frame1_gray=frame1_gray(:,110:745);
frame2_gray=rgb2gray(frame2);
frame2_gray=frame2_gray(:,110:745);

FOB1=frame2_gray(51:100,51:100);
FOB2=frame2_gray(51:100,293:342);
FOB3=frame2_gray(51:100,537:586);

FOB4=frame2_gray(215:264,51:100);
FOB5=frame2_gray(215:264,293:342);
FOB6=frame2_gray(215:264,537:586);

FOB7=frame2_gray(381:430,51:100);
FOB8=frame2_gray(381:430,293:342);
FOB9=frame2_gray(381:430,537:586);

[counts(1,:),binLocations] = imhist(FOB1);
[counts(2,:),binLocations] = imhist(FOB2);
[counts(3,:),binLocations] = imhist(FOB3);
[counts(4,:),binLocations] = imhist(FOB4);
[counts(5,:),binLocations] = imhist(FOB5);
[counts(6,:),binLocations] = imhist(FOB6);
[counts(7,:),binLocations] = imhist(FOB7);
[counts(8,:),binLocations] = imhist(FOB8);
[counts(9,:),binLocations] = imhist(FOB9);

subplot(3,3,1);
imhist(FOB1);
subplot(3,3,2);
imhist(FOB2);
subplot(3,3,3);
imhist(FOB3);
subplot(3,3,4);
imhist(FOB4);
subplot(3,3,5);
imhist(FOB5);
subplot(3,3,6);
imhist(FOB6);
subplot(3,3,7);
imhist(FOB7);
subplot(3,3,8);
imhist(FOB8);
subplot(3,3,9);
imhist(FOB9);

figure;
subplot(3,3,1);
imshow(FOB1,[]);
subplot(3,3,2);
imshow(FOB2,[]);
subplot(3,3,3);
imshow(FOB3,[]);
subplot(3,3,4);
imshow(FOB4,[]);
subplot(3,3,5);
imshow(FOB5,[]);
subplot(3,3,6);
imshow(FOB6,[]);
subplot(3,3,7);
imshow(FOB7,[]);
subplot(3,3,8);
imshow(FOB8,[]);
subplot(3,3,9);
imshow(FOB9,[]);

FOB_array(:,:,1)=FOB1;
FOB_array(:,:,2)=FOB2;
FOB_array(:,:,3)=FOB3;
FOB_array(:,:,4)=FOB4;
FOB_array(:,:,5)=FOB5;
FOB_array(:,:,6)=FOB6;
FOB_array(:,:,7)=FOB7;
FOB_array(:,:,8)=FOB8;
FOB_array(:,:,9)=FOB9;

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
FOB_array=FOB_array(:,:,sorted_index);

figure;
subplot(3,3,1);
imhist(reshape(FOB_array(:,:,1),[50 50]));
subplot(3,3,2);
imhist(reshape(FOB_array(:,:,2),[50 50]));
subplot(3,3,3);
imhist(reshape(FOB_array(:,:,3),[50 50]));
subplot(3,3,4);
imhist(reshape(FOB_array(:,:,4),[50 50]));
subplot(3,3,5);
imhist(reshape(FOB_array(:,:,5),[50 50]));
subplot(3,3,6);
imhist(reshape(FOB_array(:,:,6),[50 50]));
subplot(3,3,7);
imhist(reshape(FOB_array(:,:,7),[50 50]));
subplot(3,3,8);
imhist(reshape(FOB_array(:,:,8),[50 50]));
subplot(3,3,9);
imhist(reshape(FOB_array(:,:,9),[50 50]));

figure;
subplot(3,3,1);
imshow(reshape(FOB_array(:,:,1),[50 50]),[]);
subplot(3,3,2);
imshow(reshape(FOB_array(:,:,2),[50 50]),[]);
subplot(3,3,3);
imshow(reshape(FOB_array(:,:,3),[50 50]),[]);
subplot(3,3,4);
imshow(reshape(FOB_array(:,:,4),[50 50]),[]);
subplot(3,3,5);
imshow(reshape(FOB_array(:,:,5),[50 50]),[]);
subplot(3,3,6);
imshow(reshape(FOB_array(:,:,6),[50 50]),[]);
subplot(3,3,7);
imshow(reshape(FOB_array(:,:,7),[50 50]),[]);
subplot(3,3,8);
imshow(reshape(FOB_array(:,:,8),[50 50]),[]);
subplot(3,3,9);
imshow(reshape(FOB_array(:,:,9),[50 50]),[]);

FOB1_im2=frame1_gray(51:100,51:100);
FOB2_im2=frame1_gray(51:100,293:342);
FOB3_im2=frame1_gray(51:100,537:586);

FOB4_im2=frame1_gray(215:264,51:100);
FOB5_im2=frame1_gray(215:264,293:342);
FOB6_im2=frame1_gray(215:264,537:586);

FOB7_im2=frame1_gray(381:430,51:100);
FOB8_im2=frame1_gray(381:430,293:342);
FOB9_im2=frame1_gray(381:430,537:586);

FOB_array_im2(:,:,1)=FOB1_im2;
FOB_array_im2(:,:,2)=FOB2_im2;
FOB_array_im2(:,:,3)=FOB3_im2;
FOB_array_im2(:,:,4)=FOB4_im2;
FOB_array_im2(:,:,5)=FOB5_im2;
FOB_array_im2(:,:,6)=FOB6_im2;
FOB_array_im2(:,:,7)=FOB7_im2;
FOB_array_im2(:,:,8)=FOB8_im2;
FOB_array_im2(:,:,9)=FOB9_im2;

FOB_array_im2=FOB_array_im2(:,:,sorted_index);

difference_image=abs(FOB_array_im2-FOB_array);

figure;
subplot(3,3,1);
imshow(reshape(difference_image(:,:,1),[50 50]),[]);
subplot(3,3,2);
imshow(reshape(difference_image(:,:,2),[50 50]),[]);
subplot(3,3,3);
imshow(reshape(difference_image(:,:,3),[50 50]),[]);
subplot(3,3,4);
imshow(reshape(difference_image(:,:,4),[50 50]),[]);
subplot(3,3,5);
imshow(reshape(difference_image(:,:,5),[50 50]),[]);
subplot(3,3,6);
imshow(reshape(difference_image(:,:,6),[50 50]),[]);
subplot(3,3,7);
imshow(reshape(difference_image(:,:,7),[50 50]),[]);
subplot(3,3,8);
imshow(reshape(difference_image(:,:,8),[50 50]),[]);
subplot(3,3,9);
imshow(reshape(difference_image(:,:,9),[50 50]),[]);

movement_measure=zeros(1,9);
for i=1:9
    temp=double(reshape(difference_image(:,:,i),[50 50]));
    temp=temp(:);
    movement_measure(1,i)=sum(temp);
end
figure;
plot(movement_measure);

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

% clearvars -except FB FB_location frame1_gray frame2_gray3
H = vision.TemplateMatcher;
LOC = step(H,frame1_gray,FB);
LOC=LOC-24;
LOC=double([LOC(2),LOC(1)])
FB_location
vibration_vector=FB_location-LOC

restored_frame=frame2_gray((51:430)-vibration_vector(1),(51:586)-vibration_vector(2));
figure;
imshow(restored_frame);
figure;
imshow(frame2_gray(51:430,51:586));
figure;
imshow(frame1_gray(51:430,51:586));
