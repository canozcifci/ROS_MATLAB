
myCam = imaq.VideoDevice('linuxvideo',1,'RGB24_1280x720','ReturnedDataType','uint8'); %videoinput
    
vidPlayer = vision.DeployableVideoPlayer;
% Load control parameters
params = controlParams;
%% 
while(1) 
    %% 
    % Grab images
    img = step(myCam);

    %%
    % Object detection algorithm
    resizeScale = 0.5;
    [centerX,centerY,circleSize] = detectCircle(img,resizeScale);
    % Object tracking algorithm
    [v,w] = trackCircle(centerX,circleSize,size(img,2),params);
    
    %% 
    % Display velocity results
    fprintf('Linear Velocity: %f, Angular Velocity: %f\n',v,w);
    
    %%
    img = insertShape(img,'Circle',[centerX centerY circleSize/2],'LineWidth',2);
    step(vidPlayer,img);
        
end
