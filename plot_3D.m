%%%%
% Victor Freire <freiremelgiz@wisc.edu>
% ARC Lab Research Group <https://xu.me.wisc.edu/>
% University of Wisconsin-Madison
% Created: May 2021
%
% Plot in 3D Axes
%%%%

function [] = plot_3D(fnum,data,col,bounds,waypoints,ctrl_p,axislabels)
    % fnum Figure Number
    % data [3xk] Main data trace
    % col Color formatting for main trace
    % bounds [3x2] Axis bounds
    % waypoints [3x(l+1)] Waypoints 'or'
    % ctrl_p [3x(N+1)] Control points '-ob'
    % axislabels [1x3]
    figure(fnum)
    plot3(data(1,:),data(2,:),data(3,:),col,'LineWidth',1.5)
    hold on
    if ~isempty(waypoints)
        plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'or')
    end
    if ~isempty(ctrl_p)
        plot3(ctrl_p(1,:),ctrl_p(2,:),ctrl_p(3,:),'-ob')
    end
    if ~isempty(axislabels)
        xlabel(axislabels(1),'Interpreter','Latex');
        ylabel(axislabels(2),'Interpreter','Latex');
        zlabel(axislabels(3),'Interpreter','Latex');
    end
    grid on
    if ~isempty(bounds)
        xlim(bounds(1,:));
        ylim(bounds(2,:));
        zlim(bounds(3,:));
    end
end