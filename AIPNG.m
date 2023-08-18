%%
clear;
clc;
Vt=sqrt(0.2^2+0.1^2);
R0=sqrt(1^2+1.2^2);
theta0=atan(2);
alphat=-pi/12;
%Vr0=Vt*cos(at-theta0)-Vp*cos(d);
%Vtheta0=Vt*sin(at-theta0)-Vp*sin(d);
lambda=5;
Kd=2;
Kp=1;
ati=0.1;
atj=0.2;
a_angle=atan2(atj,ati);


f=@(t,y)[y(6)*cos(y(5)-y(2))-y(3)*cos(y(4)-y(2));
       (y(6)*sin(y(5)-y(2))-y(3)*sin(y(4)-y(2)))/y(1);       
       ( lambda*((y(6)*sin(y(5)-y(2))-y(3)*sin(y(4)-y(2)))^2)/y(1) +cos(y(2))*ati+ sin(y(2))*atj )*cos(y(4)-y(2))   +  ((-lambda*(y(6)*cos(y(5)-y(2))-y(3)*cos(y(4)-y(2)))*(y(6)*sin(y(5)-y(2))-y(3)*sin(y(4)-y(2))))/y(1) -sin(y(2))*ati + cos(y(2))*atj)*sin(y(4)-y(2));
       ( -( lambda*((y(6)*sin(y(5)-y(2))-y(3)*sin(y(4)-y(2)))^2)/y(1) +cos(y(2))*ati+ sin(y(2))*atj )*sin(y(4)-y(2))   +  ((-lambda*(y(6)*cos(y(5)-y(2))-y(3)*cos(y(4)-y(2)))*(y(6)*sin(y(5)-y(2))-y(3)*sin(y(4)-y(2))))/y(1) -sin(y(2))*ati + cos(y(2))*atj)*cos(y(4)-y(2)))/y(3);
       sqrt(ati*ati+atj*atj)*sin(a_angle-y(2))/y(6);
       sqrt(ati*ati+atj*atj)*cos(a_angle-y(2))
       y(6)*cos(y(5));
       y(6)*sin(y(5));
         ];
     [t y]=ode23(f,[0 10],[R0 theta0 0.1 0 alphat Vt 0 0]);
 


%% CT-MODE
Kd=2;
l=1;
q=1;
Vtheta=Vt*sin(alphat-y(:,2))-y(:,3).*sin(y(:,4)-y(:,2));
rdot=y(:,6).*cos(y(:,5)-y(:,2))-y(:,3).*cos(y(:,4)-y(:,2));
for k=1:1:length(Vtheta)
    if(abs(Vtheta(k))<0.01*max(abs(Vtheta)) && l==1)
        Index1=k;
        l=l+1;   
    end
    if(abs(y(k,1))<0.001*max(abs(y(:,1))) && q==1)
        Index2=k;
        q=q+1;   
    end
    
end
plot(t,rdot);
hold on;
%plot(t,y(:,1));

%plot(t(Index1),y(Index1,1),'o');
plot(t(Index1),rdot(Index1),'o');

%plot(t(Index2),y(Index2,1),'o');
plot(t(Index2),rdot(Index2),'o');
%% 
T_t=t(Index1)-y(Index1,1)/(rdot(Index1)+Kd*y(Index1,1)/2);
pred=[];
y10=[];
t10=[];
plot(0,0);
hold on;
Kd=0.2;
Kp=0.01;
prediction=[];
plot([-0.5 2],[0 0],'black');
plot([0 0],[-0.14 0.02],'black');
for k=Index1:1:Index2
    if(-y(k,1)/(rdot(k)+Kd*y(k,1)/2)>0)
    pred=[pred;-y(k,1)/(rdot(k)+Kd*y(k,1)/2)];
    %plot(k,-y(k,1)/(rdot(k)+Kd*y(k,1)/2),'o');
    pause(0.1);
    end
    opt=Index1+5;
    f=@(t1,y1)[y1(2); -Kd*y1(2)-Kp*y1(1)];

    [t1 y1]=ode45(f,[0 50],[y(k,1) rdot(k)]);
    hold on;
    plot(y(Index1,1),rdot(Index1),'o');
    plot(y(1:k,1),rdot(1:k))
    plot(y1(:,1),y1(:,2))
    xlabel('r(m)'); ylabel('v(m/s)')
    if(k>=opt)
        plot(y(opt,1),rdot(opt),'o');
    end
    pause(2);
    q=1;
    time=0;
    for i=1:1:length(t1)
        if(abs(y1(i,1)<0.01) && abs(y1(i,2)<0.01) && q==1)
            time=t1(i);
            q=2;
        end
    
    end
    if (time~=0)
    prediction=[prediction t(k)+time];
    end
    t(k)
    time
end
%plot(t(Index+1:length(t)),pred);

%% ploting of trajectory
%for i=2:1:length(t)
%Xt(i)=Xt(1)+Vt*t(i)*cos(at);
%Yt(i)=Yt(1)+Vt*t(i)*sin(at);
%end
%Xt=y(:,1).*cos(y(:,2));
%Yt=y(:,1).*sin(y(:,2));
pause(5);
Xt=y(:,7);
Yt=y(:,8);
Xp=Xt-y(:,1).*cos(y(:,2));
Yp=Yt-y(:,1).*sin(y(:,2));;
%xlim([0 3000]);
%ylim([0 3000]);
%Xp=y(:,3).*cos(y(:,4));
%Yp=y(:,3).*sin(y(:,4));

for i=1:1:length(Xt)

plot(Xt(1:i),Yt(1:i),'r');
hold on;
plot(Xp(1:i),Yp(1:i),'b');
plot(Xt(i),Yt(i),'o');
plot(Xp(i),Yp(i),'o');
xlabel('x'); ylabel('y')

pause(0.1);
hold off;
end
