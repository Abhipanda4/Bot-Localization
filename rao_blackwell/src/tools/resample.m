% resample the set of particles.
% A particle has a probability proportional to its weight to get
% selected. A good option for such a resampling method is the so-called low
% variance sampling, Probabilistic Robotics pg. 109
function newParticles = resample(particles)

numParticles = length(particles);

w = [particles.weight];

% normalize the weight
w = w / sum(w);

% consider number of effective particles, to decide whether to resample or not
useNeff = false;
%useNeff = true;
if useNeff
	neff = 1. / sum(w.^2);
	neff
	if neff > 0.5*numParticles
		newParticles = particles;
		for i = 1:numParticles
			newParticles(i).weight = w(i);
	end
	return;
end
end

newParticles = struct;

% TODO: implement the low variance re-sampling

% the cumulative sum

% initialize the step and the current position on the roulette wheel

% walk along the wheel to select the particles
r = rand(1) / 1000;
n = 1;
c = w(1);
for i = 1:numParticles
	u = r + (i - 1) * 1.0 / numParticles;
	%u
	while u > c
		n = n + 1;
		c = c + w(n);
	end
	newParticles(i) = particles(n);
	newParticles(i).weight = w(n);
end

end
