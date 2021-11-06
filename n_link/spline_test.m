t = [0,3];
y = [.5, .5];
cy = spline(t,[-1 y 1]);
xx = 0:0.01:3;
plot(t,y,'o',xx,ppval(cy,xx),'-');
der = fnder(cs,1);
ppval(der,3)
t = [0,3];
x = [0.3, 1];
cx = spline(t,[0.3 x -1]);
xx2 = 0:0.01:3.5;
plot(t,x,'o',xx,ppval(cx,xx),'-');
plot(ppval(cx,xx2),ppval(cy,xx2))
hold on 
plot(ppval(cx,xx),ppval(cy,xx))

x = 0.5:0.005:1;
x = x';
y = 0.5*ones(length(x),1);
ytmp = y;


tp = linspace(this.t,this.t+t_flight,115);
y = ppval(y_path,tp);
x = ppval(x_path,tp);
th = ppval(th_path,tp);
Qtest = [];
Q3 = q;
tic
for i = 1:length(x)
    if i == 1
        Qtest(:,i) = n_link_invKin(x(i),y(i),th(i),[1;1;.5],Q3);
    else
        Qtest(:,i) = n_link_invKin(x(i),y(i),th(i),[1;1;.5],Qtest(:,i-1));
    end
end
toc
n_link_ball_animate(linspace(0,5,length(x)),[Qtest;zeros(2,length(x))]',this.L,this.r,this.d)


x = 0:10; 
y1 = sin(x); 
y2 = 2*sin(x);
y = [y1;y2]';
xi =3; 
yi = interp1(x,y,xi) 