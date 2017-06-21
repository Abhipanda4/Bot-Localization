function newParicles = resampling(poseEstimate, prev_weight, sc)
	mu = [0, 0, 0]';
	sigma = zeros(3,3);
	delta = 0.2

	% number of points considered for computing approximate
	% gaussian distribution of proposal model
	K = 30
end
