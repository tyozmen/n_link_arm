function [val,ist,dir] = contact_n_link_ball_v2(t,Q,L,r,d)
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

% project xn and ee to the surface of the arm

xnp = xn - d*sin(th);
ynp = yn + d*cos(th);

xeep = ee(1) - d*sin(th);
yeep = ee(2) + d*cos(th);

% is the closest point on the ball to the surface of the arm on the arm?

xynp2xyeep = sqrt((xnp-xeep)^2+(ynp-yeep)^2);    % distance from xynp to xyeep

xyr2xynp = sqrt((xnp-xr)^2+(ynp-yr)^2);        % distance from xyr to xynp
xyr2xyeep = sqrt((xr-xeep)^2+(yr-yeep)^2);      % distance from xyr to xyeep

% if the closest point on the ball is on the surface of the arm then
% xynp2xyeep =  xyr2xynp + xyr2xyeep (if point C is on line segment AB then AB = AC + CB)


eqn = slope*xr + b + yshift - yr;

% this if clause assigns a value to the val function depending on the
% location of the closest point on the ball wrt the surface.
% If val = 0 then the ball hits the surface
if xeep > xnp && xr >= xnp && xr <= xeep
    flag = 1;
elseif xeep < xnp && xr <= xnp && xr >= xeep
    flag = 1;
else
    flag = 0;
end
        
    
if eqn <= 0 && flag % below
    val(1,1) = -(xyr2xyeep + xyr2xynp - xynp2xyeep);
elseif eqn > 0 && flag       % above
    val(1,1) = (xyr2xyeep + xyr2xynp - xynp2xyeep);
else
    val(1,1) = -(xyr2xyeep + xyr2xynp - xynp2xyeep);
end
val(2,1) = yb;

ist = [1; 1];                     % Stop integrating
dir = [1; -1];
end