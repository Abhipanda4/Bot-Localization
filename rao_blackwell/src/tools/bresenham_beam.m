function obstacle = bresenham_beam(mycoords, map)
% function to calculate the obstacle position along a particular direction on basis of current map
x = round(mycoords(:,1));
y = round(mycoords(:,2));
steep = (abs(y(2)-y(1)) > abs(x(2)-x(1)));

if steep, [x,y] = swap(x,y); end

	if x(1)>x(2), 
		[x(1),x(2)] = swap(x(1),x(2));
		[y(1),y(2)] = swap(y(1),y(2));
	end

delx = x(2)-x(1);
dely = abs(y(2)-y(1));
error = 0;
x_n = x(1);
y_n = y(1);
if y(1) < y(2), ystep = 1; else ystep = -1; end 
for n = 1:delx+1
	if steep,
		X(n) = x_n;
		Y(n) = y_n;
	else
		X(n) = y_n;
		Y(n) = x_n;
	end
	x_n = x_n + 1;
	error = error + dely;
	if bitshift(error,1) >= delx, % same as -> if 2*error >= delx, 
		y_n = y_n + ystep;
		error = error - delx;
	end
	if log_odds_to_prob(map(X(n), Y(n))) > 0.80
		obstacle = [X(n); Y(n)];
		return;
	end
end
obstacle = [mycoords(2,:)]';

temp = X;
X = Y;
Y = temp;

function [q,r] = swap(s,t)
% function SWAP
q = t; r = s;

