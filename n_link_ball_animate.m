function n_link_ball_animate(t_opt, X_opt, L,r_b,d)
% L is a vector of link lengths
% L = ones(2,1);
n = length(L);
% f = figure('Color','w','Renderer','zbuffer');

% axis([-(max(abs(X_opt(1,:)))+.4) max(abs(X_opt(1,:)))+.4 -2 2]);

q = X_opt(:,1:n);
x_b = X_opt(:,n+1);
y_b = X_opt(:,n+2);
f = figure('Color','w','Renderer','zbuffer');
axis([-sum(L)-.25 sum(L)+.25 -sum(L)-.25 sum(L)+.25]);
for i = 1:n

    
%     th = .06; % half-THickness of arm
    th = d;
    avals = pi*[0:.05:1];
    x(i,:) = [0 L(i) L(i)+th*cos(avals-pi/2) L(i) 0 th*cos(avals+pi/2)];
    y(i,:) = [-th -th th*sin(avals-pi/2) th th th*sin(avals+pi/2)];
    r(i,:) = (x(i,:).^2 + y(i,:).^2).^.5;
    a(i,:) = atan2(y(i,:),x(i,:));
    if i ==1
        xdraw(:,:,i) = r(i,:).*sin(a(i,:)+q(:,i));  % x pts to plot, for Link 1
        ydraw(:,:,i) = r(i,:).*cos(a(i,:)+q(:,i));  % y pts to plot, for Link 1
        xend(:,i) = L(i)*sin(q(:,i));  % "elbow" at end of Link 1, x
        yend(:,i) = L(i)*cos(q(:,i));  % "elbow" at end of Link 1, x
    else
        xdraw(:,:,i) = xend(:,i-1) + r(i,:).*sin(a(i,:)+q(:,i));  % x pts to plot, for Link 1
        ydraw(:,:,i) = yend(:,i-1) + r(i,:).*cos(a(i,:)+q(:,i));  % y pts to plot, for Link 1
        xend(:,i) = xend(:,i-1) + L(i)*sin(q(:,i));  % "elbow" at end of Link 1, x
        yend(:,i) = yend(:,i-1) + L(i)*cos(q(:,i));
    end

    f; 
    p(i) = patch(xdraw(1,:,i),ydraw(1,:,i),'b','FaceAlpha',.3); hold on %pole 1
end

b = rectangle('Position', [x_b(1)-r_b y_b(1)-r_b 2*r_b 2*r_b],'Curvature',1, 'FaceColor', [0, 1, 1, .3]);

dim = [.7 .7 .2 .2];
str=sprintf('t = %.2f',t_opt(1));
a1 = annotation('textbox',dim,'String',str,'FitBoxToText','on');
axis equal; %axis off
for i=5:length(X_opt)
    if mod(i,25)==0 || i==2 || i==length(X_opt)
        for j = 1:n
            set(p(j), 'Visible', 'off');
            p(j) = patch(xdraw(i,:,j),ydraw(i,:,j),'b','FaceAlpha',.3);
        end
        set(b, 'Position', [x_b(i)-r_b y_b(i)-r_b 2*r_b 2*r_b])
        str=sprintf('t = %.2f',t_opt(i));
        set(a1, 'String', str)        
        figure(f)
        axis([-sum(L)-.25 sum(L)+.25 -sum(L)-.25 sum(L)+.25]);
        drawnow                                	
        pause(0.005)
    end
end
end
% axis([-2.5 2.5 -1.5 1.5])