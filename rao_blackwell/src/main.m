addpath('tools')
% pkg load statistics
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
% TODO: DO NOT HARDCODE IT
probOcc  = 0.90;
probFree = 0.10;

% Cell resolution
gridSize = 0.1;

border = 30;

% mapbox defines the map boundaries
mapBox = [-10-border 35+border -35-border 35+border];

% Define the origin in map frame
offsetX = mapBox(1);
offsetY = mapBox(3);

% Map offset used when converting from world to map coordinates.
offset = [offsetX; offsetY];

% Dimensions of the map
mapSizeMeters = [mapBox(2)-offsetX, mapBox(4)-offsetY];
mapSize = ceil([mapSizeMeters/gridSize])

% motion-noise
noise = [0.05 0.1 0.05]';

% define number of particles to represent the bot
numParticles = 8;

% Initial cell occupancy probability.
prior = 0.50;
logOddsPrior = prob_to_log_odds(prior);

% The occupancy value of each cell in the map is initialized with the prior.
map = logOddsPrior * ones(mapSize(1), mapSize(2), numParticles);

% Initial state of each map, used in updating the map
initial_state = logOddsPrior * ones(mapSize);

% initialize the particles array
particles = struct;
for i = 1:numParticles
	particles(i).weight = 1. / numParticles;
	particles(i).pose = zeros(3,1);
	particles(i).history = [];
	particles(i).map = map(:,:,i);
end

% Main loop for updating map cells.
% runs once for each pose measurement obtained
for(t=1 : size(data.timestep,2))
	printf('Timestep: %d\n', t);

	%Laser scan made at time t.
	sc = laser(1,t);

	% perform prediction step for each particle pose
	% the sampled particles have new weights and poses alloted here
	particles = prediction_step(particles, data.timestep(t).odometry, noise, gridSize, offset, sc);

	% update the grid map for each particle
	[particles, laserEndPntsMapFrame] = gridmap(particles, sc, gridSize, mapBox, offset, initial_state, probOcc, probFree);

	% resample only above a certain threshold
	particles = resample(particles);

	% print the mean pose estimate
	mean_pose = zeros(size(particles(1).pose));
	for i=1:numParticles
		mean_pose += particles(i).pose;
	end
	mean_pose = mean_pose / numParticles


	% final_map = build_map(particles);
	plot_map(particles, mapBox, gridSize, offset, laserEndPntsMapFrame, t)
end
disp('----------------------------------------------------------');
disp(final_map);
