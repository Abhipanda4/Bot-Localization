function sensor_prob = sensor_model(map, sample_poses, gridSize, offset, scan, noise)
% to calculate p(z|m_t-1, x_t)
% use beam endpoint model
% from the robot pose, move towards the point indicated by laser scan direction
% till we find an obstacle


% map -> the map of the surroundings
% sample_poses -> 3xK matrix, each column representing a pose
% 			     sampled around the best estimate
% gridSize -> resolution
% offset -> for origin
% scan -> a row vector with 360 measurements about distances
% noise -> measurement noise

% precompute sigma and normalizing factor for the gaussian
sigma = 0.2;
norm_factor = 1 / sqrt(2 * pi * sigma);


alpha = 0.3;

% initialize the return array
% at the end of the loop, this array contains
% K coloumns, each giving the sensor model for one sample pose
sensor_prob = [];

% for each sample pose, compute the sensor model probability distribution
for i = 1:size(sample_poses,2)

	% Robot pose as a homogeneous transformation matrix.
	robTrans = v2t(sample_poses(:,i));

	% compute robPoseMapFrame; i.e, the cell of bot's location
	robPoseMapFrame(1:2) = world_to_map_coordinates(sample_poses(1:2,i), gridSize, offset);

	% Compute the Cartesian coordinates of the laser beam endpoints.
	% Set the third argument to 'true' to use only half the beams
	laserEndPnts = robotlaser_as_cartesian(scan, 20, true);

	% Compute the endpoints of the laser beams in the world coordinates frame.
	laserEndPnts = robTrans * laserEndPnts;

	% compute laserEndPntsMapFrame from laserEndPnts.
	% This gives a matrix of form [X;Y] for all the endpoints of laser beams
	laserEndPntsMapFrame = world_to_map_coordinates(laserEndPnts(1:2,:), gridSize, offset);

	temp = 1;
	for sc = 1:4:size(laserEndPntsMapFrame,2)
		% obstacle position based on present map
		obstacle = bresenham_beam(...
			[robPoseMapFrame(1), robPoseMapFrame(2);...
			laserEndPntsMapFrame(1,sc), laserEndPntsMapFrame(2, sc)], map);
		
		% calculate the probability
		prob = norm_factor * ...
		exp(-0.5 * gridSize * gridSize * (laserEndPntsMapFrame(:,sc) - obstacle)' * ...
		(laserEndPntsMapFrame(:,sc) - obstacle) / sigma);

		temp = temp * (prob^alpha);
	end
	
	% a list of probabilities for all x_ks around x* for gaussian computation
	sensor_prob = [sensor_prob temp];
end
end
