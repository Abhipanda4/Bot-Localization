function [particles, laserEndPntsMapFrame] = gridmap(particles, sc, gridSize, mapBox, offset, initial_state, probOcc, probFree)
	for i=1 : length(particles)
		% Robot pose at the given step
		robPose = particles(i).pose;

		% Compute the mapUpdate, which contains the log odds values to add to the map.
		[mapUpdate, laserEndPntsMapFrame] = inv_sensor_model(particles(i).map, sc, robPose, gridSize, offset, probOcc, probFree);
		mapUpdate -= initial_state;
		
		% Update the occupancy values of the affected cells.
		particles(i).map += mapUpdate;
	end
end
