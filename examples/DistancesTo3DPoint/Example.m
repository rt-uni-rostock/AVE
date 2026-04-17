% known reference points
p1 = [0; 0; 0];
p2 = [1; 0; 0];
p3 = [0; 1; 0];
p4 = [0; 0; 1];
referencePoints = [p1, p2, p3, p4];

% noisy distance measurements to unknown target (here 5 measurements per reference point)
testPoint = [2;3;4];
r1 = norm(testPoint - p1) * (1.0 + 0.001*randn(5,1));
r2 = norm(testPoint - p2) * (1.0 + 0.001*randn(5,1));
r3 = norm(testPoint - p3) * (1.0 + 0.001*randn(5,1));
r4 = norm(testPoint - p4) * (1.0 + 0.001*randn(5,1));
distances = [r1, r2, r3, r4];

% estimate
targetPoint = ave.DistancesTo3DPoint(distances, referencePoints);
fprintf('Estimation Result: [%f; %f; %f]\n', targetPoint(1), targetPoint(2), targetPoint(3));
fprintf('Ground Truth:      [%f; %f; %f]\n', testPoint(1), testPoint(2), testPoint(3));

% plot comparison
figure(1); clf; hold on;
plot3(referencePoints(1,:), referencePoints(2,:), referencePoints(3,:), 'ko', 'LineWidth', 3, 'MarkerSize', 10);
plot3(testPoint(1), testPoint(2), testPoint(3), 'bo', 'LineWidth', 3, 'MarkerSize', 10);
plot3(targetPoint(1), targetPoint(2), targetPoint(3), 'r*', 'LineWidth', 2, 'MarkerSize', 10);
legend('reference points', 'ground truth target', 'estimated target');
view(50,20); box on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');