rosshutdown;clear;clc;
% Initiliaze the ROS
rosinit
% Load control parameters

% Create ROS subscribers and publishers

lidarSub = rossubscriber("/scan");
[velPub,velMsg] = rospublisher("cmd_vel"); 
posSub = rossubscriber("/odom");

plotobj = CanHelperTurtleBotVisualizer([-11,11,-11,11]);
%  figure
%  hold on;
% grid on;grid minor;
%  axis([-15 15 -15 15])
% distanceToGoal = 0.2;


% params.spinVelocity = 0.2;       % Angular velocity (rad/s)
% params.forwardVelocity = 0.6;     % Linear velocity (m/s)
% params.backwardVelocity = -0.3;  % Linear velocity (reverse) (m/s)
% params.distanceThreshold = 0.75;   % Distance threshold (m) for turning


 tic;
   while toc < 20
   
      % Collect information from laser scan
      scan = receive(lidarSub);
      position = receive(posSub);
      q3=position.Pose.Pose.Orientation.Z;
      q4=position.Pose.Pose.Orientation.W;
      
      quat = position.Pose.Pose.Orientation;
      angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
      theta = rad2deg(angles(1));
      
      pose = [position.Pose.Pose.Position.X,position.Pose.Pose.Position.Y,quat2angle([q4 0 0 q3])];
      
      laserData = readCartesian(scan) * [0 1; -1 0];
%         plot(laserData);
%          plotData(plotobj,pose,laserData);
% 
      while(theta<0) 
            theta = theta + 360;
       end     
       
      [M,N] = min(scan.Ranges);
      
%       minDist = M;
     
      
      a = theta + N -180;
      while(a>=360)
          a = a-360;
      end    
      while(a<0)
          a = a+360;
      end    
      P = N+180;
      while(P>360)
          P = P-360;
      end
      T = scan.Ranges(P);
      O = (M+T)/2;
       
      
      x1 = pose(1) + cosd(a)*(O-M);
      y1 = pose(2) + sind(a)*(O-M);
      
      wp = [x1 y1];
      distanceThreshold = 0.2;
      distanceToGoal = norm([pose(1) pose(2)] - [wp(1,1) wp(1,2)]);
%       
%       r = rosrate(10);
%       reset(r);
% while(distanceToGoal < distanceThreshold)   
 
     
     ctrlr=robotics.PurePursuit;
     ctrlr.Waypoints = wp;
     ctrlr.DesiredLinearVelocity = 0.2;
     ctrlr.MaxAngularVelocity = 0.15;
     ctrlr.LookaheadDistance = 4;
%      while(distanceToGoal > 0.2) 
                 

%        t1  = pose(1)^2 + pose(2)^2; % robotun orjine uzaklığı 
%        t2 = x1^2 + y1^2;            % orta noktanın orjine uzaklığı
%         
%        
%        if(t1 < t2 ) % robot sağa dönmeli 
%            if(distanceToGoal > 0)
%            angvel = -distanceToGoal*0.25;
%            end
%             if(distanceToGoal < 0)
%            angvel = distanceToGoal*0.25;
%            end
%        end
%        if(t1 > t2) % robot sola dönmeli 
%             if(distanceToGoal > 0)   
%             angvel = distanceToGoal*0.25;
%             end
%             if(distanceToGoal < 0)
%                 angvel = -distanceToGoal*0.25;
%             end    
%        end
%           
       
linvel = 0.2;
   
    count = 0;

    [linvel,angvel] = ctrlr(pose);
    angvel = min(0.5*angvel*distanceToGoal,2);
%     if(distanceToGoal < 0.35)
%         if(count == 0)
%         angvel = angvel*-1*3;
%         count = count+1;
%         end
%     end
%     if(distanceToGoal > 0.3)
%         count = 0;
%     end
    fprintf("distance = %d\n",distanceToGoal);
      fprintf("angvel = %d\n",angvel);
%     fprintf("count = %d\n",count);
     
      velMsg.Linear.X = linvel;
      velMsg.Angular.Z = angvel;
      send(velPub,velMsg);
%      end
%       waitfor(r);
%       
% end  
%       
%          m0 = [x1;y1;0;-1];
%         kf_m = ozgur(m0);
%         velMsg.Linear.X = sqrt(kf_m(3)^2+kf_m(4)^2);
%         send(velPub,velMsg);
       
      
      
       th = pose(3)-pi/2;
       dataWorld = laserData*[cos(th) sin(th);-sin(th) cos(th)] ...
      + repmat(pose(1:2),[numel(laserData(:,1)),1]);
      
        plot_Data(plotobj,pose,laserData,x1,y1);




      %walPos = RobCoor + cor.Cartesian;            % X düzleminde giderken
      %walPos = RobCoor + y;                        % Y için
      %plot(walPos(:,1),walPos(:,2))        
      
      ranges = double(scan.Ranges);
%       plot(scan)
       
%       front_1 = min(min(ranges(1:10),3.5));
%       front_2 = min(min(ranges(350:360),3.5));
%       front_left = min(min(ranges(11:50),3.5));
%       left = min(min(ranges(51:90),3.5));
%       rear = min(min(ranges(91:269),3.5));
%       right = min(min(ranges(270:309),3.5));
%       front_right = min(min(ranges(310:349),3.5));
%       
%       if ((floor(front_1) && floor(front_2)) == 1)
%           disp('Going straight...')
%           velMsg.Linear.X = 1;
%           velMsg.Angular.Z = 0;
%       end
%         if((front_left - front_right) < 0)
%           disp('Turning right...')
%           velMsg.Linear.X = 0.5;
%           velMsg.Angular.Z = -0.15;
%         else((front_right - front_left) < 0);
%           disp('Turning left...')
%           velMsg.Linear.X = 0.5;
%           velMsg.Angular.Z = 0.15;
%         end
%       send(velPub,velMsg);
%    break;
  
%       %angles = double(scan.readScanAngles); 
%       %plot(scan);
%       data = readCartesian(scan);
%       x = data(:,1);
%       y = data(:,2);
%       % Compute distance of the closest obstacle
%       dist = sqrt(x.^2 + y.^2);
%       minDist = min(dist);  
%       disp(minDist)
%       % Command robot action
%       if minDist < params.distanceThreshold
%           % If close to obstacle, back up slightly and spin          
%           velMsg.Linear.X = params.backwardVelocity;  %geri git saçma burası pls
%           velMsg.Angular.Z = params.spinVelocity;   % sola dön 
%           send(velPub,velMsg);
%       else
%           % Continue on forward path
%           velMsg.Linear.X = params.forwardVelocity;
%           velMsg.Angular.Z = 0;
%           send(velPub,velMsg);
%          
%       end   
%       send(velPub,velMsg);
      end
    disp('Stopped.')
    velMsg.Linear.X = 0;
    velMsg.Angular.Z = 0;     
    send(velPub,velMsg);