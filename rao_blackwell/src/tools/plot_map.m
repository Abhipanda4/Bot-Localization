function plot_map(particles, mapBox, gridSize, offset, laserEndPntsMapFrame, t)

	close all
	% figure 
	figure(1, "visible", "off");
	axis(mapBox);
	hold on;

	% calculate final map as weighted sum of each particles map
	map = zeros(size(particles(1).map));
	for i=1:length(particles)
		map = map + (log_odds_to_prob(particles(i).map) * particles(i).weight);
	end
	map = map';
	imshow(ones(size(map)) - map)
	s = size(map)(1:2); 
        set(gcf, "position", [50 50 s*5]) 
        set(gca, "position", [.05 .05 .9 .9]) 
	

	% Calculate trajectory of each particle and plot it
	traj = [];
	for i=1:size(particles, 2)
		traj = [traj, [particles(i).history(1,:); particles(i).history(2,:)]];
	end
	traj = world_to_map_coordinates(traj, gridSize, offset);
	plot(traj(1,:),traj(2,:),'g')
	robPoseMapFrame = [];
	for i=1:size(particles, 2)
		tmp = world_to_map_coordinates(particles(i).pose(1:2,:), gridSize, offset);
		robPoseMapFrame = [robPoseMapFrame tmp];
	end
	plot(robPoseMapFrame(1,:),robPoseMapFrame(2,:),'bo','markersize',2,'linewidth',1)

	plot(laserEndPntsMapFrame(1,:),laserEndPntsMapFrame(2,:),'ro','markersize',1)

   	filename = sprintf('../plots/gridmap_%03d.png', t);
	print(filename, '-dpng');
	close all;

end
