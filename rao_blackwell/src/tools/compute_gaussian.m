function [mu, sigma, eta] =...
compute_gaussian(sample_poses, best_fit, map, gridSize, offset, scan, noise)

sensor_readings = sensor_model(map, sample_poses, gridSize, offset, scan, noise);
motion_readings = motion_model(best_fit, noise, sample_poses);

mu = 0;
eta = 0;
sigma = zeros(size(sample_poses,1));
for i=1:size(sample_poses,2)
	mu += motion_readings(1,i) * sensor_readings(1,i) * sample_poses(:,i);
	eta += motion_readings(1,i) * sensor_readings(1,i);
end

mu = mu / eta;

for i=1:size(sample_poses,2)
	sigma += (sample_poses(:,i) - mu) * (sample_poses(:,i) - mu)' * motion_readings(1,i) * sensor_readings(1,i); 
end

sigma = sigma / eta;
end
