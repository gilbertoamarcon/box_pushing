function generate_mov_clip(problem_num,frame_rate)

PROBLEM_PRE     = 'problem';
FIG_PREFIX_POS	= '/figs/FIG';
MOV_NAME_POS	= '/figs/MOV.avi';
 
FIG_PATTERN     = '%s%03d.fig';
WIDTH           = 320;
HEIGHT          = 240;

FIG_PREFIX      = sprintf('%s%d%s',PROBLEM_PRE,problem_num,FIG_PREFIX_POS);
MOV_NAME        = sprintf('%s%d%s',PROBLEM_PRE,problem_num,MOV_NAME_POS);

% Writting video
outputVideo = VideoWriter(MOV_NAME);
outputVideo.FrameRate = frame_rate;
open(outputVideo);
for s = 1:1e10
    filename = sprintf(FIG_PATTERN,FIG_PREFIX,s);
    try
        h = openfig(filename,'invisible');
        set(h, 'Position', [0 0 WIDTH HEIGHT]);
        frame = getframe(h);
    catch
        break;
    end
    writeVideo(outputVideo,frame);
end
close(outputVideo);

exit;