%% The algorithm
% 1: Robot moves forward to be close to a wall
% 2: Start following left wall
% 3: Don't get to close to the left wall
% 4: If something in front, turn until clear
%% Setup
clear;clc;
rosinit
plotobj = CanHelperTurtleBotVisualizer([-5,5,-5,5]);
% Create ROS subscribers and publishers
lidarSub = rossubscriber("/scan");
[velPub,velMsg] = rospublisher("cmd_vel"); 
posSub = rossubscriber("/odom");
% figure
% hold on;
% grid on;grid minor;
% axis([-11 11 -11 11])
%% Process
tic;
while toc < 15
    
    % Collect information from laser scan
    scan = receive(lidarSub);
    position = receive(posSub);
    
    % To get pose, use quaternion
    q3=position.Pose.Pose.Orientation.Z;
    q4=position.Pose.Pose.Orientation.W;   
    quat = position.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = rad2deg(angles(1));  
    pose = [position.Pose.Pose.Position.X,position.Pose.Pose.Position.Y,quat2angle([q4 0 0 q3])];
   
    % Plot the walls and robot pose
    laserData = readCartesian(scan) * [0 1; -1 0];
    plot_Data(plotobj,pose,laserData);
   
    % Lidar sections
    ranges = double(scan.Ranges);
    front_1 = min(min(ranges(1:10),3.5));
    front_2 = min(min(ranges(350:360),3.5));
    min_front = min(front_1,front_2);
    front_left = min(min(ranges(11:50),3.5));
    left = min(min(ranges(51:90),3.5));
    rear = min(min(ranges(91:269),3.5));
    right = min(min(ranges(270:309),3.5));
    front_right = min(min(ranges(310:349),3.5));
    
    % Follow left wall
    if(min_front < 0.3)
        disp ('Obstacle!turning....') 
        velMsg.Linear.X = 0;
        velMsg.Angular.Z = -0.1;
    elseif((left - right) < -0.25)
        disp('Turning right...')
        velMsg.Linear.X = 0.15;
        velMsg.Angular.Z = -0.1;
    elseif(right - left < -0.25)
        disp('Turning left...')
        velMsg.Linear.X = 0.15;
        velMsg.Angular.Z = 0.1;
    end
    send(velPub,velMsg);
end    
% Stop the robot
  
disp('Stopped.')
velMsg.Linear.X = 0;
velMsg.Linear.Y = 0;
velMsg.Linear.Z = 0;
velMsg.Angular.X = 0;
velMsg.Angular.Y = 0;
velMsg.Angular.Z = 0;
send(velPub,velMsg);
% end     
rosshutdown;    
  
