% generate test points
z = linspace(0,10,60);
poses = [4*cos(z); 3*sin(z); z];

for i = 1:100
    % run line simplification
    thresholdPosition = (1.5*cos(i/20))^4;
    thresholdAngle = (1.5*cos(i/20))^4;
    indices = ave.geometry.DouglasPeuckerSE2(poses, thresholdPosition, thresholdAngle);

    % plot
    figure(1); clf; hold on;
    plot3(poses(1,:), poses(2,:), poses(3,:), 'k*-');
    if(~isempty(indices))
        plot3(poses(1,indices), poses(2,indices), poses(3,indices), 'o-');
    end
    axis image; box on; grid on; view(20,50); xlabel('x in m'); ylabel('y in m'); zlabel('\psi in rad'); title(['thresholdPosition: ', num2str(thresholdPosition), ' thresholdAngle: ', num2str(thresholdAngle)]); drawnow();
end
