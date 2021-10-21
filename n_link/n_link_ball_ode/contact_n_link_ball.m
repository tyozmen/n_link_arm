function [val,ist,dir] = contact_n_link_ball(t,Q,L,r,d)
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

% is the closest point on the ball to the surface of the arm on the arm?
eqn = slope*xr + b + yshift - yr;
val = eqn;
% if abs(eqn) < e-2 && xn >= xr && xr >= ee(1) % if eqn = 0 and xr is in between the x values of the last link then the ball hits the arm
%     val = 1;
% % elseif eqn == 0 && xn >= xr && xr >= ee(1) % if eqn = 0 and xr isn't in between the x values of the last link then nothing happens
% %     val = 1;
% % else
% %     val = eqn;
% end
   % if eqn = 0 then the ball hits the arm
ist = 1;                     % Stop integrating
dir = 0;
end