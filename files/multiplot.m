clear all;
close all;
clc;

PROBLEM_NUM     = 26;
PROBLEM_PRE     = 'problem';
FIG_PREFIX_POS	= '/figs/FIG';
FIG_PREFIX      = sprintf('%s%d%s',PROBLEM_PRE,PROBLEM_NUM,FIG_PREFIX_POS);
FIG_PATTERN     = '%s%03d.fig';

I = 4;
J = 6;

ax_vec = zeros(I,J);
for i=1:I
    for j=1:J
        index = J*(i-1)+j;
        filename = sprintf(FIG_PATTERN,FIG_PREFIX,index);
        try
            h = openfig(filename,'reuse','invisible');
            ax_vec(i,j) = gca;
        catch
            break;
        end
    end
end

h = figure(I*J+1);
for i=1:I
    for j=1:J
        index = J*(i-1)+j;
        try
            sub = subplot(I,J,index);
            copyobj(get(ax_vec(i,j),'children'),sub);
            axis equal;
            axis off;
        catch
            delete(sub);
            break;
        end
    end
end