rosshutdown;clear;clc;
% Initiliaze the ROS
rosinit
% Load control parameters

% Create ROS subscribers and publishers

lidarSub = rossubscriber("/scan");
[velPub,velMsg] = rospublisher("cmd_vel"); 
posSub = rossubscriber("/odom");

plotobj = ExampleHelperTurtleBotVisualizer([-20,20,-20,20]);

% figure
% hold on;
% grid on;grid minor;
% % axis([-20 20 -20 20])


% params.spinVelocity = 0.2;       % Angular velocity (rad/s)
% params.forwardVelocity = 0.6;     % Linear velocity (m/s)
% params.backwardVelocity = -0.3;  % Linear velocity (reverse) (m/s)
% params.distanceThreshold = 0.75;   % Distance threshold (m) for turning


 tic;
  while toc < 60
      % Collect information from laser scan
      scan = receive(lidarSub);
      position = receive(posSub);
      q3=position.Pose.Pose.Orientation.Z;
      q4=position.Pose.Pose.Orientation.W;
      
      
      pose = [position.Pose.Pose.Position.X,position.Pose.Pose.Position.Y,quat2angle([q4 0 0 q3])];
      
%       cor = lidarScan(scan);
%       
%       RobCoor = [position.Pose.Pose.Position.X,position.Pose.Pose.Position.Y];
%       y = [cor.Cartesian(:,2),cor.Cartesian(:,1)];    % y düzleminde giderken bunu kullan yani x coulumnla y yi değiştir.

        laserData = readCartesian(scan) * [0 1; -1 0];
%         plot(laserData);
         plotData(plotobj,pose,laserData);



      
      %walPos = RobCoor + cor.Cartesian;            % X düzleminde giderken
      %walPos = RobCoor + y;                        % Y için
      %plot(walPos(:,1),walPos(:,2))        
      
      ranges = double(scan.Ranges);
%       plot(scan)
       
      front_1 = min(min(ranges(1:10),3.5));
      front_2 = min(min(ranges(350:360),3.5));
      front_left = min(min(ranges(11:50),3.5));
      left = min(min(ranges(51:90),3.5));
      rear = min(min(ranges(91:269),3.5));
      right = min(min(ranges(270:309),3.5));
      front_right = min(min(ranges(310:349),3.5));
      
      if ((floor(front_1) && floor(front_2)) == 1)
          disp('Going straight...')
          velMsg.Linear.X = 0.6;
          velMsg.Angular.Z = 0;
      end
        if((front_left - front_right) < 0)
          disp('Turning right...')
          velMsg.Linear.X = 0.4;
          velMsg.Angular.Z = -0.15;
        else((front_right - front_left) < 0);
          disp('Turning left...')
          velMsg.Linear.X = 0.4;
          velMsg.Angular.Z = 0.15;
        end
      send(velPub,velMsg);
      
     
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