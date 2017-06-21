function particles = gridmap(particles, scan, gridSize, offset, probOcc, probFree, logOddsPrior)
for(t=1 : size(particles,1))
	% Robot pose at the given step
	robPose = [particle(i).pose];

	% Compute the mapUpdate, which contains the log odds values to add to the map.
	[mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(particle(i).map, sc, robPose, gridSize, offset, probOcc, probFree);

	mapUpdate -= logOddsPrior * ones(size(particles(i).map));
	% Update the occupancy values of the affected cells.
	particles(i).map += mapUpdate;

	% Plot current map and robot trajectory so far.
	plot(particles(i).map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t);

end
end
