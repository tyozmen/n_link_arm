function [x_imp,y_imp,l_imp,th] = n_link_impact_point(Q,L,r,d)
% get the impact point at the moment of collision

n = length(L);
ee = n_link_fwdKin(Q(1:n),L);
xb = Q(n+1);
yb = Q(n+2);



xn = ee(1) - L(n)*sin(Q(n));    % x coordinate of nth link
yn = ee(2) - L(n)*cos(Q(n));    % y coordinate of the nth link
slope = cos(Q(n))/sin(Q(n));

% y = slope*x+b  line equation of the arm

b = yn - slope*xn;

th = atan(slope); % angle of the surface wrt x axis

yshift = d/cos(th);

% equation for the surface of the arm
% y = slope*x + b + yshift

% closest point on the ball to the surface of the arm
xr = xb + sin(th)*r;
yr = yb - cos(th)*r;

eqn = slope*xr + b + yshift - yr;

% project xr onto the center line of the arm

x_imp = xr + d*sin(th);
y_imp = yr - d*cos(th);
l_imp = sqrt((xn - x_imp)^2+(yn-y_imp)^2);

end