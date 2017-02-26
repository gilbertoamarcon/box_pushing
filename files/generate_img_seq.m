function generate_img_seq(problem_num)

PROBLEM_PRE     = 'problem';
MAP_FILE_POS	= '/map.csv';
PLAN_FILE_POS	= '/plan.csv';
FIG_PREFIX_POS	= '/figs/FIG';

MAP_FILE        = sprintf('%s%d%s',PROBLEM_PRE,problem_num,MAP_FILE_POS);
PLAN_FILE       = sprintf('%s%d%s',PROBLEM_PRE,problem_num,PLAN_FILE_POS);
FIG_PREFIX      = sprintf('%s%d%s',PROBLEM_PRE,problem_num,FIG_PREFIX_POS);
FIG_PATTERN     = '%s%03d.fig';
GOAL_APHA       = 0.20;
GOAL_COLOR      = [1-GOAL_APHA 1-GOAL_APHA 1];
B_SIZE          = 0.95;
DISP_PATH       = 1;

% Map loading
map = fliplr(csvread(MAP_FILE));

% Map loading
map = csvread(MAP_FILE);

% Plan execution loading
try
    file = fileread(PLAN_FILE);
catch 
    exit;
end
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
    
    % Clearing figure     
    clf;
    hold on;
        
    % Map
    for i =1:size(map,1)
        for j =1:size(map,2)
            r_y = i-1/2;
            r_x = j-1/2;
            pos = [r_x r_y 1 1];
            if map(i,j)
                rectangle('Position',pos,'FaceColor','k','EdgeColor','none');
            else
                rectangle('Position',pos,'FaceColor','w','EdgeColor','none');
            end
        end
    end

    % Plotting goal box positions
    for i =1:size(box,3)
        y = box(1,num_steps,i)';
        x = box(2,num_steps,i)';
        r_y = y+0.5+(1-B_SIZE)/2;
        r_x = x+0.5+(1-B_SIZE)/2;
        pos = [r_x r_y B_SIZE B_SIZE];
        rectangle('Position',pos,'FaceColor',GOAL_COLOR,'EdgeColor','none');
        text(x+1,y+1,char(i+64),'Color','w','FontSize',14,'HorizontalAlignment','center');
    end

    % Plotting robot path
    if DISP_PATH
        for i =1:size(robot,3)
            py = robot(1,1:s,i)'+1;
            px = robot(2,1:s,i)'+1;
            plot(px,py,'r');
        end
    end

    % Plotting box positions
    for i =1:size(box,3)
        y = box(1,s,i)';
        x = box(2,s,i)';
        r_y = y+0.5+(1-B_SIZE)/2;
        r_x = x+0.5+(1-B_SIZE)/2;
        pos = [r_x r_y B_SIZE B_SIZE];
        rectangle('Position',pos,'FaceColor','b','EdgeColor','none');
        text(x+1,y+1,char(i+64),'Color','w','FontSize',14,'HorizontalAlignment','center');
    end

    % Plotting robot positions
    for i =1:size(robot,3)
        y = robot(1,s,i)';
        x = robot(2,s,i)';
        r_y = y+0.5+(1-B_SIZE)/2;
        r_x = x+0.5+(1-B_SIZE)/2;
        pos = [r_x r_y B_SIZE B_SIZE];
        rectangle('Position',pos,'FaceColor','r','EdgeColor','none');
        text(x+1,y+1,char(i+47),'Color','w','FontSize',14,'HorizontalAlignment','center');
    end

    axis equal;
    axis tight;
    
    % Saving to file
    filename = sprintf(FIG_PATTERN,FIG_PREFIX,s);
    saveas(gcf,filename,'fig');

end

exit;
