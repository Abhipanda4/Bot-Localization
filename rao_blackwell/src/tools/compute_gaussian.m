function [mu, sigma, eta] =...
compute_gaussian(sample_poses, best_fit, map, gridSize, offset, scan, noise)

% function to computee the approximate gaussian for proposal model
% sample_poses -> the poses sampled around x*
% best_fit -> x*
% map -> map carried by the particle

correction = 0.05;
sensor_readings = sensor_model(map, sample_poses, gridSize, offset, scan, noise);
sensor_readings = max(sensor_readings, correction * ones(size(sensor_readings)));
motion_readings = motion_model(best_fit, sample_poses);

mu = 0;
eta = 0;
sigma = zeros(size(sample_poses,1));
predicted_poses = [];
proposal_products = sensor_readings .* motion_readings;
for i=1:columns(sample_poses)
	predicted_poses = [predicted_poses, proposal_products(1,i) * sample_poses(:,i)];
end

for i=1:columns(sample_poses)
	mu += predicted_poses(:,i);
	% mu(3) = normalize_angle(mu(3));
	eta += proposal_products(1,i);
end

mu = mu / eta;
mu(3) = normalize_angle(mu(3));

for i=1:size(sample_poses,2)
	sigma += (predicted_poses(:,i) - mu) * (predicted_poses(:,i) - mu)'; 
end

sigma = sigma / eta;
end
