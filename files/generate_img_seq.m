clear all;
close all;
clc;

PROBLEM_NUM     = 2;
PROBLEM_PRE     = 'problem';
MAP_FILE_POS	= '/map.csv';
PLAN_FILE_POS	= '/plan.csv';
FIG_PREFIX_POS	= '/figs/FIG';

MAP_FILE        = sprintf('%s%d%s',PROBLEM_PRE,PROBLEM_NUM,MAP_FILE_POS);
PLAN_FILE       = sprintf('%s%d%s',PROBLEM_PRE,PROBLEM_NUM,PLAN_FILE_POS);
FIG_PREFIX      = sprintf('%s%d%s',PROBLEM_PRE,PROBLEM_NUM,FIG_PREFIX_POS);
FIG_PATTERN     = '%s%03d.png';
GOAL_APHA       = 0.20;
DISP_PATH       = 1;


BLOCK_SIZE      = 1000;

% Map loading
map = fliplr(csvread(MAP_FILE));
BLOCK_SIZE      = 1000;

% Map loading
map = csvread(MAP_FILE);

% Plan execution loading
file = fileread(PLAN_FILE);
data = strread(file,'%s','delimiter','\n');
num_steps = length(data);

% Parsing plan history positions
for i =1:num_steps

    % Parsing data
    set = strread(data{i},'%s','delimiter',':');

    % Robot positions
    pos = regexp(set{2}, '\d+,\d+,','match');
    for j =1:length(pos)
        robot(:,i,j) = strread(pos{j},'%d','delimiter',',')';
    end

    % Box positions
    pos = regexp(set{1}, '\d+,\d+,','match');
    for j =1:length(pos)
        box(:,i,j) = strread(pos{j},'%d','delimiter',',')';
    end
end

% Generating image sequences
delete(strcat(FIG_PREFIX,'*'));
for s=1:num_steps
    
    % Map
    hold all;
    colormap([1 1 1; 0 0 0 ]);
    image(map .* 255);

    % Plotting robot path
    if DISP_PATH
        for i =1:size(robot,3)
            py = robot(1,1:s,i)'+1;
            px = robot(2,1:s,i)'+1;
            plot(px,py,'r');
        end
    end

    % Plotting goal box positions
    for i =1:size(box,3)
        y = box(1,num_steps,i)'+1;
        x = box(2,num_steps,i)'+1;
        scatter(x,y,BLOCK_SIZE,'sw','filled');
        scatter(x,y,BLOCK_SIZE,'sb','filled','MarkerFaceAlpha',GOAL_APHA);
        text(x,y,char(i+64),'Color','w','FontSize',14,'HorizontalAlignment','center');
    end

    % Plotting box positions
    for i =1:size(box,3)
        y = box(1,s,i)'+1;
        x = box(2,s,i)'+1;
        scatter(x,y,BLOCK_SIZE,'sb','filled');
        text(x,y,char(i+64),'Color','w','FontSize',14,'HorizontalAlignment','center');
    end

    % Plotting robot positions
    for i =1:size(robot,3)
        y = robot(1,s,i)'+1;
        x = robot(2,s,i)'+1;
        scatter(x,y,BLOCK_SIZE,'sr','filled');
        text(x,y,char(i+47),'Color','w','FontSize',14,'HorizontalAlignment','center');
    end

    axis equal;
    axis tight;
    drawnow;
    
    % Saving to file
    filename = sprintf(FIG_PATTERN,FIG_PREFIX,s);
    hgexport(gcf,filename,hgexport('factorystyle'), 'Format', 'png'); 

end

