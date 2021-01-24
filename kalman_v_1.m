vr=1; %velocity of robot
vd=0.9; %velocity of destination
tr(1)=pi; %initial first orientation of robot

xr(1)=vr*cos(tr(1)); %initial X positon of robot
yr(1)=vr*sin(tr(1)); %initial Y positon of robot
wr(1)=vr*tr(1);

td(1)=pi; %initial orientation of destination
xd(1)=vd*cos(td(1)); %initial X positon of destination
yd(1)=vd*sin(td(1)); %initial Y positon of destination
wd(1)=vd*td(1);

tr(2)=pi;
xr(2)=vr*cos(tr(2));
yr(2)=vr*sin(tr(2));
wr(2)=vr*tr(2);

td(2)=pi;
xd(2)=vd*cos(td(2));
yd(2)=vd*sin(td(2));
wd(2)=vd*td(2);

X_R(:,1) = [xr(1);yr(1);wr(1)];
Y_R(:,1) = [xr(1);yr(1);wr(1)];
X_D(:,1) = [xd(1);yd(1);wd(1)];
Y_D(:,1) = [xd(1);yd(1);wd(1)];

X_R(:,2) = [xr(2);yr(2);wr(2)];
Y_R(:,2) = [xr(2);yr(2);wr(2)];

X_D(:,2) = [xd(2);yd(2);wd(2)];
Y_D(:,2) = [xd(2);yd(2);wd(2)];

X_C(:,:,1) = 0.84*eye(3); %initial covariance constant of the robot
X_CD(:,:,1) = 0.74*eye(3); %initial covariance constant of the destination

X_C(:,:,2) = 0.93*eye(3); %initial covariance constant of the robot
X_CD(:,:,2) = 0.93*eye(3); %initial covariance constant of the destination

X_W = 0.3*eye(3); %measurement noise defined

H = 0.492*eye(3); %state transition matrix of the robot 

HD = 0.5*eye(3); %state transition matrix of the destination

arr=1;
m=1;
n=1;
%Kalman filter implementation
for i=2:1:50
%predict
X_D(:,i+1) = HD*X_D(:,i) + X_D(:,i);
Y_D(:,i+1)= X_D(:,i+1) + rand(3,1); %the observed/measured value for moving

wd(i+1) = HD(3,3)*vd*td(i) + wd(i);
td(i+1) = wd(i+1)/2;
xd(i+1) = HD(1,1)*vd*cos(td(i+1)) + xd(i);
yd(i+1) = HD(2,2)*vd*sin(td(i+1)) + yd(i);

X_D(:,i+1) = [xd(i+1);yd(i+1);wd(i+1)];
Y_D(:,i+1) = [xd(i+1);yd(i+1);wd(i+1)] + [0.2;0.1;0.1];

C_D = [X_D(:,i-1) X_D(:,i) X_D(:,i+1)];
CD(:,:,m)=cov(C_D);
X_CD(:,:,arr)=cov(C_D); %calculated covariance for moving destination

X_CD(:,:,arr)=(HD*X_CD(:,:,arr)*HD')+X_CD(:,:,arr);
%new predicted value for covariance of moving destination

tr(i+1) = td(i+1);
wr(i+1) = 2*tr(i+1);
xr(i+1) = H(1,1)*vr*cos(tr(i+1)) + xr(i); 
yr(i+1) = H(2,2)*vr*sin(tr(i+1)) + yr(i);

%xr(i+1) = H(1,1)*2*cos(tr(i+1)) + xr(i);
%yr(i+1) = H(2,2)*2*sin(tr(i+1)) + yr(i);

X_R(:,i+1) = [xr(i+1);yr(i+1);wr(i+1)];
Y_R(:,i+1) = [xr(i+1);yr(i+1);wr(i+1)] + [0.2;0.1;0.1];

C_R = [X_R(:,i-1) X_R(:,i) X_R(:,i+1)];
CR(:,:,n)=cov(C_R);
X_C(:,:,arr)=cov(C_R);
%calculated covariance
X_C(:,:,arr) = (H*X_C(:,:,arr)*H') + X_C(:,:,arr);

%update target
X_D(:,i+1)=X_D(:,i+1)+((X_CD(:,:,arr).*eye(3))*HD'/(HD*(X_CD(:,:,arr).*eye(3))*HD' + X_W)*(Y_D(:,i+1) - HD*X_D(:,i+1)));
%updated value of X_D: destination

%covariance of moving destination
X_CD(:,:,arr)=(X_CD(:,:,arr).*eye(3))+((X_CD(:,:,arr).*eye(3))*HD')/((HD*(X_CD(:,:,arr).*eye(3))*HD')+X_W)*HD*(X_CD(:,:,arr).*eye(3));
%updated value of X_CD:

%update robot
X_R(:,i+1) = X_R(:,i+1) +
((X_C(:,:,arr).*eye(3))*H'/(H*(X_C(:,:,arr).*eye(3))*H' +
X_W)*(Y_R(:,i+1) - H*X_R(:,i+1)));
%updated value of X_R
%covariance of moving target
X_C(:,:,arr)=(X_C(:,:,arr).*eye(3))+(((X_C(:,:,arr).*eye(3))*H')/((H*(X_C(:,:,arr).*eye(3))*H')+X_W)*H*(X_C(:,:,arr).*eye(3)));

%updated value of X_C
m=m+1;
n=n+1;
arr=arr+1;
end
subplot(2,1,1),
p=0;
for p=1:1:arr
surfc(X_CD(:,:,p));
hold on
end

hold off
subplot(2,1,2),
plot(X_R(1,:),X_R(2,:),'r');
hold on
plot(X_R(1,:),X_R(2,:),'+');
plot(X_D(1,:),X_D(2,:),'g');
plot(X_D(1,:),X_D(2,:),'*');
plot(X_D(1,1),X_D(2,1),'O');
plot(X_R(1,1),X_R(2,1),'O');
hold off



 


