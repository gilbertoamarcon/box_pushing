clear all;
close all;
clc;

FIG_PREFIX	= 'files/FIG';   
FIG_PATTERN	= '%s%03d.png';
MOV_NAME	= 'files/MOV.avi';
MOV_FRATE	= 4;

% Writting video
outputVideo = VideoWriter(MOV_NAME);
outputVideo.FrameRate = MOV_FRATE;
open(outputVideo);
for s = 1:1e10
    filename = sprintf(FIG_PATTERN,FIG_PREFIX,s);
    try
        img = imread(filename);
    catch
        break;
    end
    writeVideo(outputVideo,img)
end
close(outputVideo);