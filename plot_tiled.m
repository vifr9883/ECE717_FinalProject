%%%%
% Victor Freire <freiremelgiz@wisc.edu>
% ARC Lab Research Group <https://xu.me.wisc.edu/>
% University of Wisconsin-Madison
% Created: June 2021
%
% Plot n traces in n tiles according to layout arg
%%%%
function [] = plot_tiled(fnum,data,col,t,axislabels,layout)
    % data: [nxk]
    % axislabels: n elements
    % layout: [rows,cols], rows+cols = n
    exist = ishandle(fnum);
    figure(fnum)
    hold on
    if ~isempty(layout) && ~exist
        tiledlayout(layout(1), layout(2))
    end
    n = size(data,1); % Number of traces
    for i = 1:n
        nexttile(i)
        plot(t,data(i,:),col,'LineWidth',1.0)
        hold on
        grid on
        xlim([0,round(t(end),2)]);
        if ~isempty(axislabels)
            xlabel('$t$ $[s]$','Interpreter','Latex')
            ylabel(axislabels(i),'Interpreter','Latex')
        end
    end
end