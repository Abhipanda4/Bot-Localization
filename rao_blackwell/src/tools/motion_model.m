function values = motion_model(best_fit, points)
% best_fit -> the best estimated pose of particle
% noise -> motion noise
% points -> a set of poses around best fit to compute approximate gaussian
	mu = repmat(best_fit, 1, size(points, 2));
	motionNoise = 0.1;
	sigma = [motionNoise, 0, 0;
			0, motionNoise, 0;
			0, 0, motionNoise/10];
	
	difference = points - mu;
	difference(3,:) = normalize_angle(difference(3,:));
	exponents = [];
	for i=1:size(points,2)
		exponents = [exponents, difference(:,i)' * inv(sigma) * difference(:,i)];
	end

	values = 1 / sqrt(2 * pi * det(sigma)) * exp(exponents * -0.5);
end
