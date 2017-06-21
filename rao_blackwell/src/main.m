addpath('tools')
more off
close all
clear all

% Load laser scans and robot poses.
load("../data/laser")

% read the odometry commands
data = read_data('../data/odometry.dat');
% data.timestep(i).odometry.x     -> x motion in timestep i
% data.timestep(i).odometry.y     -> y motion in timestep i
% data.timestep(i).odometry.theta -> theta motion in timestep i

% Probabilities related to the laser range finder sensor model.
probOcc  = 0.90;
probFree = 0.10;

% Cell resolution
gridSize = 0.5;

border = 20;

% mapbox defines the map boundaries
mapBox = [-10-border 35+border -35-border 35+border];

% like the origin
offsetX = mapBox(1);
offsetY = mapBox(3);

% Map offset used when converting from world to map coordinates.
offset = [offsetX; offsetY];

mapSizeMeters = [mapBox(2)-offsetX mapBox(4)-offsetY];
mapSize = ceil([mapSizeMeters/gridSize]);

% motion-noise
noise = [0.2, 0.2, 0.1]';

% define number of particles to represent the bot
numParticles = 10

% Initial cell occupancy probability.
prior = 0.50;
logOddsPrior = prob_to_log_odds(prior);

% The occupancy value of each cell in the map is initialized with the prior.
map = logOddsPrior * ones(mapSize, mapSize, numParticles);

% used in updating the map
initial_state = logOddsPrior * ones(mapSize, mapSize);

% initialize the particles array
particles = struct;
for i = 1:numParticles
	particles(i).weight = 1. / numParticles;
	particles(i).pose = zeros(3,1);
	particles(i).history = cell();
	particles(i).map = map(:,:,i);
end

% Main loop for updating map cells.
% runs once for each pose measurement obtained
for(t=1 : size(data.timestep,2))
	%Laser scan made at time t.
	sc = laser(1,t);

	% perform prediction step for each particle pose
	% the sampled particles have new weights and poses alloted here
	particles = prediction_step(particles, data.timestep(t).odometry, noise);

	% update the grid map for each particle
	particles = gridmap(particles, scan, gridSize, offset, probOcc, probFree, logOddsPrior);

	% resample only above a certain threshold, here N/2
	particles = resample(particles);
end
