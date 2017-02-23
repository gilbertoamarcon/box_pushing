clear all;
close all;
clc;

PROBLEM_NUM     = 9;
PROBLEM_PRE     = 'problem';
FIG_PREFIX_POS	= '/figs/FIG';
MOV_NAME_POS	= '/figs/MOV.avi';
 
FIG_PATTERN     = '%s%03d.png';
MOV_FRATE       = 4;

FIG_PREFIX      = sprintf('%s%d%s',PROBLEM_PRE,PROBLEM_NUM,FIG_PREFIX_POS);
MOV_NAME        = sprintf('%s%d%s',PROBLEM_PRE,PROBLEM_NUM,MOV_NAME_POS);

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