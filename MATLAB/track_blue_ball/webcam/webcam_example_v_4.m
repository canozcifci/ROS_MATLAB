%% Webcam Example by Can
% Setup
myCam = imaq.VideoDevice('linuxvideo',1,'RGB24_1280x720','ReturnedDataType','uint8');
    
vidPlayer = vision.DeployableVideoPlayer;
% Load control parameters
params = controlParams;
%% Control the rate of execution
% Runs in 30 Hz
r = rateControl(30);
reset(r)
%% 
while(1) 
    %% 
    % Grab images
    img = step(myCam);

    %%
    % Object detection algorithm
    resizeScale = 0.5;
    [centerX,centerY,circleSize,isObjectDetected] = detectCircle(img,resizeScale);
    detected = [centerX, centerY];
    
    [tracked] = kalmanCan(detected,params,isObjectDetected);
    
    [v,w] = trackCircle(tracked(1),circleSize,size(img,2),params);
    
    %% 
    % Display velocity results
    fprintf('Linear Velocity: %f, Angular Velocity: %f\n',v,w);
    %%
    img = insertShape(img,'Circle',[centerX centerY circleSize/2],'LineWidth',2);
    step(vidPlayer,img);
    %% 
    waitfor(r);
        
end
