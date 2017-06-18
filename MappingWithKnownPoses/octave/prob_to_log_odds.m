function l=prob_to_log_odds(p)
	% Convert proability values p to the corresponding log odds l.
	% p could be a scalar or a matrix.

	% TODO: compute l.

	temp = ones(size(p, 1), size(p, 2));
	l = log(p ./ (temp - p));

end
