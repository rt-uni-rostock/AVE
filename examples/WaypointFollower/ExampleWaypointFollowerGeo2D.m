% test data
waypoints = deg2rad([
52.53866866445692, 13.331035169159835;
52.54022223087463, 13.334035984063716;
52.54037399019203, 13.33594022108587;
52.539451445157766, 13.339295618456347
]');
p0 = deg2rad([52.53939639284977, 13.333034079711394]');
p1 = deg2rad([52.53879806134964, 13.34253933338141]');
altitude = 70;
pos = [p0; altitude];
R = 12.0;

% load waypoints
wpf = ave.WaypointFollowerGeo2D(32);
wpf.LoadWaypoints(waypoints);

% run and plot
figure(1); clf; hold on;
plot(wpf, 'ko-', 'LineWidth', 1.5);
grid on; box on; view(90,-90); xlabel('latitude'); ylabel('longitude'); pbaspect([1,1/cos(0.916985),1]);
for i = 1:50
    [lookAheadPoint, bearingToLookAheadPoint, lineAngleAtLookAheadPoint, idxLine] = wpf.Step(pos, R);

    plot(pos(1), pos(2), 'k.');
    plot(lookAheadPoint(1), lookAheadPoint(2), 'r*');
    title(['idxLine=',num2str(idxLine)]);
    drawnow();
    pause(0.1);

    pos(1:2) = pos(1:2) + 0.5 * (lookAheadPoint - pos(1:2));
end
