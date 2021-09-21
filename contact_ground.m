function [val,ist,dir] = contact_ground(t,Q,r)
val = Q(2)-r;    % hitting the ground
ist = 1;                                        % Stop integrating
dir = 0;
end

