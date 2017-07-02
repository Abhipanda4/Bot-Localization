function map = build_map(particles)
    map = zeros(size(particles(1).map));
	for i=1:size(particles)
		map = map + (particles(i).map * particles(i).weight);
	end
    map = log_odds_to_prob(map);
end