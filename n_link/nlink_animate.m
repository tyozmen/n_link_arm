function nlink_animate(t_opt, X_opt, L)
% L is a vector of link lengths
% L = ones(2,1);
n = length(L);
% f = figure('Color','w','Renderer','zbuffer');

% axis([-(max(abs(X_opt(1,:)))+.4) max(abs(X_opt(1,:)))+.4 -2 2]);

q = X_opt(:,1:n);
f = figure('Color','w','Renderer','zbuffer');
axis([-sum(L)-.25 sum(L)+.25 -sum(L)-.25 sum(L)+.25]);
for i = 1:n

    
    th = .06; % half-THickness of arm

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



dim = [.7 .7 .2 .2];
str=sprintf('t = %.2f',t_opt(1));
a1 = annotation('textbox',dim,'String',str,'FitBoxToText','on');
axis equal; %axis off
for i=5:length(X_opt)
    if mod(i,5)==0 || i==2 || i==length(X_opt)
        for j = 1:n
            set(p(j), 'Visible', 'off');
            p(j) = patch(xdraw(i,:,j),ydraw(i,:,j),'b','FaceAlpha',.3);
        end
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