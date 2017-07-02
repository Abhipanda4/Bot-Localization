function particles = prediction_step(particles, u, noise, gridSize, offset, scan)
% Updates the particles by drawing from the motion model

% Noise parameters:
% Assume Gaussian noise in each of the three parameters of the motion model.
numParticles = length(particles);

for i = 1:numParticles

	% append the old position to the history of the particle
	particles(i).history = [particles(i).history, particles(i).pose];

	% most probable pose estimate for the new particle(best fit based on motion model)
	best_fit = particles(i).pose + [u.x; u.y; u.theta];
	best_fit(3,:) = normalize_angle(best_fit(3,:));

	% now estimate an approximate gaussian distribution for this model
	% number of sampling points
	K = 20;
	sample_poses = zeros(3, K);
	delta = 0.02;
	for j = 1 : K
		sample_poses(:, j) =  best_fit + delta * [1 - 2 * rand(); 1 - 2 * rand();  1 - 2 * rand()];
	end
	sample_poses(3,:) = normalize_angle(sample_poses(3,:));

	[mu, sigma, eta] = ...
	compute_gaussian(sample_poses, best_fit, particles(i).map, gridSize, offset, scan, noise);

	% draw random sample from this distribution
	particles(i).pose = mu;
	particles(i).pose(3) = normalize_angle(particles(i).pose(3));

	% update the weights
	particles(i).weight = particles(i).weight * eta;
end
w = [particles.weight];
w = w / sum(w);
for n=1:numParticles
	particles(n).weight = w(n);
end


end