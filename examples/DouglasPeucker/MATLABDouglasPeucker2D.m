% generate test points
y = linspace(0,10,60);
points = [3*sin(y); y];

for i = 1:100
    % run line simplification
    threshold = (1.5*cos(i/20))^4;
    indices = ave.geometry.DouglasPeucker2D(points, threshold);

    % plot
    figure(1); clf; hold on;
    plot(points(1,:), points(2,:), 'k*-');
    if(~isempty(indices))
        plot(points(1,indices), points(2,indices), 'o-');
    end
    axis image; box on; grid on; view(90,-90); xlabel('x in m'); ylabel('y in m'); title(['threshold: ', num2str(threshold)]); drawnow();
end