function generate_gif_clip(problem_num,frame_rate)

PROBLEM_PRE     = 'problem';
FIG_PREFIX_POS	= '/figs/FIG';
MOV_NAME_POS	= '/figs/GIF.gif';
 
FIG_PATTERN     = '%s%03d.fig';

FIG_PREFIX      = sprintf('%s%d%s',PROBLEM_PRE,problem_num,FIG_PREFIX_POS);
MOV_NAME        = sprintf('%s%d%s',PROBLEM_PRE,problem_num,MOV_NAME_POS);

% Writting video
for s = 1:1e10
    filename = sprintf(FIG_PATTERN,FIG_PREFIX,s);
    try
        h = openfig(filename,'invisible');
        ax = gca;
        set(ax,'Position',[0 0 1 1],'units','normalized');
        frame = getframe(h);
    catch
        break;
    end
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,16);
    if s == 1
        imwrite(imind,cm,MOV_NAME,'gif','DelayTime',1/frame_rate, 'Loopcount',inf);
    else
        imwrite(imind,cm,MOV_NAME,'gif','DelayTime',1/frame_rate,'WriteMode','append');
    end
    
end

exit;