function bball_ground_animate(t_opt, X_opt,r)





f = figure('Color','w','Renderer','zbuffer');
axis([-(max(abs(X_opt(:,1)))+2) (max(abs(X_opt(:,1)))+2) -.5 (max(abs(X_opt(:,2)))+.4)]);
x = X_opt(:,1);
y = X_opt(:,2);
w = X_opt(:,3);
% % cart
% x2 = [x-.3 x+.3 x+.3 x-.3];
% y2 = [-.2 -.2 .2 .2 ];

%draw the cart-pole initial condition

% f = figure('Color','w','Renderer','zbuffer');
% axis([-(max(abs(X_opt(1,:)))+.4) max(abs(X_opt(1,:)))+.4 -2 2]);
% p1 = patch(x1draw(1,:),y1draw(1,:),'b','FaceAlpha',.3); hold on %pole 1
% p2 = patch(x2draw(1,:),y2draw(1,:),'r','FaceAlpha',.3); %hold on %pole 2
% c1 = rectangle('Position',[x(1)-.25 -.15 .5 .3],'Curvature',0.2, 'FaceColor', [1, 0, 0, .3]); %cart
b1 = rectangle('Position', [x(1)-r y(1)-r 2*r 2*r],'Curvature',1, 'FaceColor', [0, 1, 1, .3]); %mass 1
% m2 = rectangle('Position', [x1end(1)+L2*sin(q2(1))-.05 y1end(1)+L2*cos(q2(1))-.05 .1 .1],'Curvature',1, 'FaceColor', [0, 1, 1, .3]); %mass 2
yline(0);
% xline(0,'-.');
dim = [.7 .7 .2 .2];
str=sprintf('t = %.2f',t_opt(1));
a1 = annotation('textbox',dim,'String',str,'FitBoxToText','on');
axis equal; %axis off
for i=5:length(X_opt)
    if mod(i,1)==0 || i==2 || i==length(X_opt)
        
%         set(b1, 'Visible', 'off');

        set(b1, 'Position', [x(i)-r y(i)-r 2*r 2*r])
%         set(m2, 'Position', [x1end(i)+L2*sin(q2(i))-.05 y1end(i)+L2*cos(q2(i))-.05 .1 .1])
        str = sprintf('t = %.2f',t_opt(i));
        set(a1, 'String', str)  
        figure(f)
        hold on
        plot(x(1:i),y(1:i))
        axis([-(max(abs(X_opt(:,1)))+2) (max(abs(X_opt(:,1)))+2) -.5 (max(abs(X_opt(:,2)))+.4)]);
        drawnow                                	
        pause(0.005)
    end
end
end
% axis([-2.5 2.5 -1.5 1.5])