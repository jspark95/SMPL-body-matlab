frame_num = 120;
for i = 1:frame_num
    tmp = ['snapshot' int2str(i) '.png'];
    %tmp = [int2str(i) '.png'];
    imageNames{i} = tmp;
end
% imageNames = dir(fullfile('mtcnn','*.jpg'));
% imageNames = {imageNames.name}';
outputVideo = VideoWriter(fullfile('test'));
outputVideo.FrameRate = 24;
open(outputVideo);

%for ii = 1:length(imageNames)
for ii = 24:84
   img = imread(fullfile(imageNames{ii}));
   writeVideo(outputVideo,img)
end

close(outputVideo);