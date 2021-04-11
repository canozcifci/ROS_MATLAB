%% Webcam Example by Can
% Setup
myCam = imaq.VideoDevice('linuxvideo',1,'RGB24_1280x720','ReturnedDataType','uint8');
    
vidPlayer = vision.DeployableVideoPlayer;
% Load control parameters
params = controlParams;
isTrackInitialized = false;
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
    % Object tracking algorithm
    [v,w] = trackCircle(centerX,circleSize,size(img,2),params);
    
    %% 
    % Display velocity results
    %fprintf('Linear Velocity: %f, Angular Velocity: %f\n',v,w);
    
    %%
    % Annotate image and update the video player
    img = insertShape(img,'Circle',[centerX centerY circleSize/2],'LineWidth',2);
    step(vidPlayer,img);
    %%
    % Kalman Filter
    if ~isTrackInitialized
        if isObjectDetected
            % Initialize track when the ball is detected for the first time
            params.motionModel = 'ConstantVelocity'; % switch from ConstantAcceleration
                                                     % to ConstantVelocity
            % After switching motion models, drop noise specification entries
            % corresponding to acceleration.
            params.initialEstimateError = params.initialEstimateError(1:2);
            params.motionNoise          = params.motionNoise(1:2);
            
            kalmanFilter = configureKalmanFilter(params.motionModel, ...
            detected, params.initialEstimateError, ...
            params.motionNoise, params.measurementNoise);
            
            isTrackInitialized = true;
            
            tracked = correct(kalmanFilter, detected);
        else
            tracked = [];
        end
    else
        % Use the Kalman filter to track the ball.
        if isObjectDetected
            % Reduce the measurement noise by calling predict followed by
            % correct.
            predict(kalmanFilter);
            tracked = correct(kalmanFilter, detected);
            %fprintf('Predicted X: %f, Predicted Y: %f\n',tracked(1),tracked(2));
        else
            % Predict the ball's location
            tracked = predict(kalmanFilter);
        end
    end % trackInitialized
        
end % while
