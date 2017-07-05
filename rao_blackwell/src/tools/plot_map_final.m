function plot_map_final(particles, mapBox, gridSize, offset)

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
    for i=1:size(map,1)
        for j=1:size(map,2)
            if map(i,j) >= 0.65
                map(i,j) = 1;
            else if map(i,j) < 0.35
                map(i,j) = 0;
			else
				map(i,j) = 0.5;
            end
        end
    end

	imshow(ones(size(map)) - map)
	s = size(map)(1:2); 
        set(gcf, "position", [50 50 s*5]) 
        set(gca, "position", [.05 .05 .9 .9]) 

   	filename = sprintf('../plots/final_map.png');
	print(filename, '-dpng');
	close all;

end
