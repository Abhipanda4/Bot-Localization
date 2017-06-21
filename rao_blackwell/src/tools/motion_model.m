function values = motion_model(best_fit, noise, points)
% best_fit -> the best estimated pose of particle
% noise -> motion noise
% points -> a set of poses around best fit to compute approximate gaussian
	mu = repmat(best_fit, 1, size(points, 2));
	sigma = noise * noise';

	power = (points - mu)' * inv(sigma) * (points - mu);

	values = 1 / sqrt(2 * pi * det(sigma)) * exp(power * -0.5);
end
