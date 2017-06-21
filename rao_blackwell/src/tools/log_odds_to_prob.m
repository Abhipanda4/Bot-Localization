function p = log_odds_to_prob(l)
	% Convert log odds l to the corresponding probability values p.
	% l could be a scalar or a matrix.

	% TODO: compute p.

	temp = ones(size(l, 1), size(l, 2));
	p = temp ./ (temp + exp( -1 * l));

end
