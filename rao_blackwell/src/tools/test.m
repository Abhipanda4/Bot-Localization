load("../data/laser")

% Extract robot poses:
% Nx3 matrix where each row is in the form: [x y theta]
poses = [laser.pose];
poses = reshape(poses, 3, size(poses,2)/3)';

% find the odometry commands by subtracting successive entries in pose
for i=2:size(poses,1)
	odometry(i - 1, :) = poses(i, :) - poses(i - 1, :);
end
odometry
