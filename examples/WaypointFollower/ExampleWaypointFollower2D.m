% test data
waypoints = [-6,6,6,-6,-6,6,6,-6,-6,-4,-4,-2,-2,0,0,2,2,4,4,6,6; -6,-6,-4,-4,-2,-2,0,0,6,6,2,2,6,6,2,2,6,6,2,2,6];
pos = [-4;-1];
R = 1.7;

% load waypoints
wpf = ave.WaypointFollower2D(32);
wpf.LoadWaypoints(waypoints);

% run and plot
figure(1); clf; hold on;
plot(wpf, 'ko-', 'LineWidth', 1.5);
grid on; box on; axis image; xlim([-10, 10]); ylim([-10, 10]); view(90,-90); xlabel('x in m'); ylabel('y in m');
for i = 1:100
    [lookAheadPoint, bearingToLookAheadPoint, lineAngleAtLookAheadPoint, idxLine] = wpf.Step(pos, R);

    PlotCircle(pos, R);
    PlotUnitLine(pos, bearingToLookAheadPoint, '#345');
    PlotUnitLine(lookAheadPoint, lineAngleAtLookAheadPoint, '#e12');
    plot(lookAheadPoint(1), lookAheadPoint(2), 'r*');
    title(['idxLine=',num2str(idxLine)]);
    drawnow();
    pause(0.1);

    pos = pos + 0.5 * (lookAheadPoint - pos);
end

function PlotCircle(center, radius)
    angle = linspace(0, 2*pi, 100);
    x = center(1) + radius * cos(angle);
    y = center(2) + radius * sin(angle);
    plot(x, y, 'LineWidth', 1.5, 'Color', '#ddd');
    plot(center(1), center(2), 'o', 'Color', '#bbb', 'LineWidth', 1.5);
end

function PlotUnitLine(point, angle, color)
    p = point + [cos(angle); sin(angle)];
    plot([point(1), p(1)],[point(2), p(2)],'LineWidth',2,'Color',color);
end
