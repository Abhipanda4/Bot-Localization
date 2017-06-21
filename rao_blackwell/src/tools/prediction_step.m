function particles = prediction_step(particles, u, noise, gridSize, offset, scan)
% Updates the particles by drawing from the motion model

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% these parameters are the sigma for the 0 mean gaussians
xNoise = noise(1);
yNoise = noise(2);
rNoise = noise(3);

numParticles = length(particles);

for i = 1:numParticles

	% append the old position to the history of the particle
	particles(i).history{end+1} = particles(i).pose;

	% most probable pose estimate for the new particle(best fit based on motion model)
	best_fit = particles(i).pose + [u.x; u.y; u.theta];
	best_fit(3,:) = normalize_angle(best_fit(3,:));

	% now estimate an approximate gaussian distribution for this model

	% number of sampling points
	K = 50
	sample_poses = zeros(3, K);
	delta = 0.3;
	for j = 1 : K
		sample_poses(:, j) =  best_fit + [rand() * delta; rand() * delta; rand() * delta];
	end

	[mu, sigma, eta] = ...
	compute_gaussian(sample_poses, best_fit, particles(i).map, gridSize, offset, scan, noise)

	% draw random sample from this distribution
	particles(i).pose = mvnrnd(mu, sigma);

	% update the weights
	particles(i).weight = particles(i).weight * eta;

end

end
