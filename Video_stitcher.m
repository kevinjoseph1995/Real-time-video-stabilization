outputVideo = VideoWriter('C:\Users\Kevin Joseph\Desktop\Video Stabilization\stablized_video.avi');
outputVideo.FrameRate = 28;
open(outputVideo);
for frame_no = 1:300
   str=strcat('C:\Users\Kevin Joseph\Desktop\Video Stabilization\image_bin\',num2str(frame_no),'.jpg');
   img = imread(str);
   writeVideo(outputVideo,img)
end
close(outputVideo);