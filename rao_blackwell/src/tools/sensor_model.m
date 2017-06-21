% to calculate p(z|m_t-1, x_t)
% use beam endpoint model

% from the robot pose, move towards the point indicated by laser scan direction
% the closest obstacle is the mean

function sensor_prob = sensor_model(map, sample_poses, gridSize, offset, scan, noise)
for i = 1:size(sample_poses,2)
	sensor_prob = [];
	% Robot pose as a homogeneous transformation matrix.
	robTrans = v2t(sample_poses(:,i));

	% compute robPoseMapFrame; i.e, the cell of bot's location
	robPoseMapFrame(1:2) = world_to_map_coordinates(sample_poses(1:2,i), gridSize, offset);

	% Compute the Cartesian coordinates of the laser beam endpoints.
	% Set the third argument to 'true' to use only half the beams
	laserEndPnts = robotlaser_as_cartesian(scan, 30, false);

	% Compute the endpoints of the laser beams in the world coordinates frame.
	laserEndPnts = robTrans * laserEndPnts;

	% compute laserEndPntsMapFrame from laserEndPnts.
	laserEndPntsMapFrame = world_to_map_coordinates(laserEndPnts(1:2,:), gridSize, offset);
	laserEndPntsMapFrame(3,:) = laserEndPnts(3,:);

	% precompute sigma and normalizing factor for the gaussian
	sigma = noise * noise';
	inv_sigma = inv(sigma);
	norm_factor = 1 / sqrt(2 * pi) * det(sigma));

	for sc = 1:columns(laserEndPntsMapFrame)
		total_prob = [];
		[unoccupied, obstacle] = bresenham_beam([robPoseMapFrame(1), robPoseMapFrame(2); laserEndPntsMapFrame(1,sc), laserEndPntsMapFrame(2, sc)]);

		obstacle = [obstacle'; sample_poses(3,i)];

		% calculate the probability
		prob = norm_factor * ...
		exp(-0.5 * gridSize * gridSize * (sample_poses(:,i) - obstacle)' * inv_sigma * ...
		(sample_poses(:,i) - obstacle));

		total_prob = [total_prob; prob];
	end
	
	% a list of probabilities for all x_ks around x* for gaussian computation
	sensor_prob = [sensor_prob, prod(total_prob)];
end
end
