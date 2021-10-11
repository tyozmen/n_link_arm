function [val,ist,dir] = contact_link_1_dof(t,Q,r,d)
val = (Q(1)+d)-(Q(2)-r);        % hitting the ground
ist = 1;                    % Stop integrating
dir = 0;
end