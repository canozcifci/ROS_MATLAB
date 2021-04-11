%% Track blue with kalman by Can
rosshutdown;clear;clc;
rosinit;
% Create ROS subscribers and publishers
imgSub = rossubscriber('/camera/rgb/image_raw');
receive(imgSub,10); % Wait to receive first message
[velPub,velMsg] = rospublisher('/cmd_vel');

% Create video player for visualization
vidPlayer = vision.DeployableVideoPlayer;

% Load control parameters
params = controlParams;
% Control the rate of execution
% Runs in 30 Hz
r = rateControl(30);
reset(r)
%%
while(1) 
    % Grab images
    img = readImage(imgSub.LatestMessage);
    
    % Object detection algorithm
    resizeScale = 0.5;
    [centerX,centerY,circleSize,isObjectDetected] = detectCircle(img,resizeScale);
    detected = [centerX, centerY];
    % Kalman Filter
    [tracked] = kalmanCan(detected,params,isObjectDetected);
    
    % Object tracking algorithm
    [v,w] = trackCircle(tracked(1),circleSize,size(img,2),params);

    velMsg.Linear.X = v;
    velMsg.Angular.Z = w;
    send(velPub,velMsg);
    
    img = insertShape(img,'Circle',[centerX centerY circleSize/2],'LineWidth',2);
    step(vidPlayer,img);
    waitfor(r); 
end
