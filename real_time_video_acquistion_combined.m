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
Hcumulative = (eye(3));
global_motion_vector=[0 0];
H = vision.TemplateMatcher;
H.ROIInputPort=1;


% Start acquiring frames.
start(vid)

while(vid.FramesAvailable >= 0)
    data = getdata(vid,2);      
    prev_frame=rgb2gray(data(:,:,:,2));
    current_frame=rgb2gray(data(:,:,:,1));
    %***************************************************************************
    ptThresh = 0.1;
    pointsA = detectFASTFeatures(prev_frame, 'MinContrast', ptThresh,'MinQuality',0.3);
    pointsB = detectFASTFeatures(current_frame, 'MinContrast', ptThresh,'MinQuality',0.3);
    %***************************************************************************
    [featuresA, pointsA] = extractFeatures(prev_frame,(pointsA));
    [featuresB, pointsB] = extractFeatures(current_frame, (pointsB));
    indexPairs = matchFeatures(featuresA, featuresB,'Method','Approximate');
    pointsA = pointsA(indexPairs(:, 1), :);
    pointsB = pointsB(indexPairs(:, 2), :);
    flag=0;
    if pointsA.Count>3 && pointsB.Count>3
        [tform, pointsBm, pointsAm] = estimateGeometricTransform(pointsB, pointsA, 'affine','MaxNumTrials',500);
        % Extract scale and rotation part sub-matrix.
        H1 = tform.T;
        R = H1(1:2,1:2);        
        % Compute theta from mean of two possible arctangents
        theta = mean([atan2(R(2),R(1)) atan2(-R(3),R(4))]);
        % Compute scale from mean of two stable mean calculations
        % scale = mean(R([1 4])/cos(theta));
        scale=1;
        % Translation remains the same:
        translation = (H1(3, 1:2));
        % Reconstitute new s-R-t transform:
        HsRt =( [[scale*[cos(theta) -sin(theta); sin(theta) cos(theta)]; ...
          0 0], [0 0 1]']);
        Hcumulative = HsRt * Hcumulative;    
        current_frame_no_rotation = imwarp(current_frame,affine2d(Hcumulative),'OutputView',imref2d(size(current_frame)));
        global_motion_vector_prev=global_motion_vector;
        global_motion_vector=-round([translation(2),translation(1)]);
        %Correction    
        k=0.95 ;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)%TWEAKABLE PARAMETER
        beta=0.015;%0.015%TWEAKABLE PARAMETER
        alpha=1;%TWEAKABLE PARAMETER (between 0 and 1)
        gamma=1;%TWEAKABLE PARAMETER (between 0 and 1)
        FMV=round(k*FMV+...
        (gamma*(alpha*global_motion_vector+(1-alpha)*global_motion_vector_prev))+...
        ((1-gamma)*(global_motion_vector-beta*constant_motion_vector_I)));
        constant_motion_vector_I=constant_motion_vector_I+FMV;    
        frame_crop_size=100;
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
        flag=1;
        
    else 
        %Cropping Current Frame
        FOB_size=50;%TWEAKABLE PARAMETER too small and you incur huge motion estimation loss; too large and frame rate drops
        temp=(FOB_size+frame_crop_size);
        cropped_frame=current_frame(temp+1:(size(current_frame,1)-temp),temp+1:(size(current_frame,2)-temp));
        middle_index_1=round(size(cropped_frame,1)/2);
        middle_index_2=round(size(cropped_frame,2)/2);    
        %Locating and storing the observation Blocks
        FOB_array(:,:,1)=cropped_frame(1:FOB_size,1:FOB_size);
        FOB_array(:,:,2)=cropped_frame(1:FOB_size,middle_index_2-(round(FOB_size/2)-1):middle_index_2+round(FOB_size/2));
        FOB_array(:,:,3)=cropped_frame(1:FOB_size,size(cropped_frame,2)-(FOB_size-1):size(cropped_frame,2));    
        FOB_array(:,:,4)=cropped_frame(middle_index_1-(round(FOB_size/2)-1):middle_index_1+round(FOB_size/2),1:FOB_size);
        FOB_array(:,:,5)=cropped_frame(middle_index_1-(round(FOB_size/2)-1):middle_index_1+round(FOB_size/2),middle_index_2-(round(FOB_size/2)-1):middle_index_2+round(FOB_size/2));
        FOB_array(:,:,6)=cropped_frame(middle_index_1-(round(FOB_size/2)-1):middle_index_1+round(FOB_size/2),size(cropped_frame,2)-(FOB_size-1):size(cropped_frame,2));    
        FOB_array(:,:,7)=cropped_frame(size(cropped_frame,1)-(FOB_size-1):size(cropped_frame,1),1:FOB_size);
        FOB_array(:,:,8)=cropped_frame(size(cropped_frame,1)-(FOB_size-1):size(cropped_frame,1),middle_index_2-(round(FOB_size/2)-1):middle_index_2+round(FOB_size/2));
        FOB_array(:,:,9)=cropped_frame(size(cropped_frame,1)-(FOB_size-1):size(cropped_frame,1),size(cropped_frame,2)-(FOB_size-1):size(cropped_frame,2));

        FB_location=[1,1;
                     1,middle_index_2-(round(FOB_size/2)-1)
                     1,size(cropped_frame,2)-(FOB_size-1)
                     middle_index_1-(round(FOB_size/2)-1),1
                     middle_index_1-(round(FOB_size/2)-1),middle_index_2-(round(FOB_size/2)-1)
                     middle_index_1-(round(FOB_size/2)-1),size(cropped_frame,2)-(FOB_size-1)
                     size(cropped_frame,1)-(FOB_size-1),1
                     size(cropped_frame,1)-(FOB_size-1),middle_index_2-(round(FOB_size/2)-1)
                     size(cropped_frame,1)-(FOB_size-1),size(cropped_frame,2)-(FOB_size-1)];

        for i=1:9
            [counts(i,:),~] = imhist(FOB_array(:,:,i));
        end

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
        FB=reshape(FOB_array(:,:,9),[FOB_size FOB_size]);
        temp=(FOB_size+frame_crop_size);
        FB_location_temp=FB_location(9,:)+temp;
        ROI_width=5;%TWEAKABLE PARAMETER
        ROI=[FB_location_temp(2)-ROI_width,FB_location_temp(1)-ROI_width,2*ROI_width+FOB_size,2*ROI_width+FOB_size];
        [LOC] = step(H,(prev_frame),(FB),(ROI));
        LOC=LOC-(round(FOB_size/2)-1);
        LOC=double([LOC(2),LOC(1)]);    
        global_motion_vector_prev=global_motion_vector;
        global_motion_vector=(FB_location_temp-LOC)
        %Correction    
        k=0.95 ;%0.95(closer to 1 smoothens, closer to 0 retains original motion characteristics)%TWEAKABLE PARAMETER
        beta=0.015;%0.015%TWEAKABLE PARAMETER
        alpha=1;%TWEAKABLE PARAMETER (between 0 and 1)
        gamma=1;%TWEAKABLE PARAMETER (between 0 and 1)
        FMV=round(k*FMV+...
        (gamma*(alpha*global_motion_vector+(1-alpha)*global_motion_vector_prev))+...
        ((1-gamma)*(global_motion_vector-beta*constant_motion_vector_I)));
        constant_motion_vector_I=constant_motion_vector_I+FMV;    
        frame_crop_size=100;
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
                
    end
    if flag==1
        stabilized_frame=current_frame_no_rotation((frame_crop_size+1:(size(current_frame_no_rotation,1)-frame_crop_size))...
        +FMV(1),(frame_crop_size+1:(size(current_frame_no_rotation,2)-frame_crop_size))+FMV(2));
    
    else
        stabilized_frame=current_frame((frame_crop_size+1:(size(current_frame,1)-frame_crop_size))+FMV(1),...
        (frame_crop_size+1:(size(current_frame,2)-frame_crop_size))+FMV(2));
    end
    step(videoPlayer, (stabilized_frame));

end
