% resample the set of particles.
% A particle has a probability proportional to its weight to get
% selected. A good option for such a resampling method is the so-called low
% variance sampling, Probabilistic Robotics pg. 109


function newParticles = resample(particles)

numParticles = length(particles);
w = [particles.weight];

% consider number of effective particles, to decide whether to resample or not
useNeff = true;
if useNeff
	neff = 1. / sum(w.^2);
	if neff > 0.5 * numParticles 
		newParticles = particles;
		for i = 1:numParticles
			newParticles(i).weight = w(i);
		end
		return;
	end
end

newParticles = struct;

% implement the low variance re-sampling
r = rand(1) / numParticles;
n = 1;
c = w(1);
for i = 1:numParticles
	u = r + (i - 1) / numParticles;
	while u > c
		n = n + 1;
		c = c + w(n);
	end
	disp('RESAMPLING');
	newParticles(i) = particles(n);
	newParticles(i).weight = w(n);
end
w = [newParticles.weight];
w = w / sum(w);
for n=1:numParticles
	newParticles(n).weight = w(n);
end
end
