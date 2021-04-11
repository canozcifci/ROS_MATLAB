function params = controlParams

    params.Ts = 0.1;           % Sample time

    params.bufSize = 5;        % Filter buffer size
    params.maxDisp = 300;      % Max object displacement [pixels]
    params.minSize = 50;       % Min object size [pixels]
    params.maxSize = 500;      % Max object size [pixels]
    params.maxCounts = 5;      % Max outlier counts before stopping

    params.linVelGain = 2e-3;  % Linear control gain
    params.angVelGain = 2e-4;  % Angular control gain
    params.maxLinVel = 0.5;    % Max linear speed
    params.maxAngVel = 0.5;   % Max angular speed

    params.posDeadZone = 30;   % Steering control marker position dead zone [pixels] 
    params.targetSize = 300;   % Linear speed control target blob size [pixels]
    params.sizeDeadZone = 30;  % Linear speed control size dead zone [pixels]
    params.speedRedSize = 100; % MiMikalmannimumkalmannimum pixel value before turning speed is ramped down
    
    params.motionModel           = 'ConstantAcceleration';   % Motion model
    params.initialLocation       = 'Same as first detection';% Initial location of the blob
    params.initialEstimateError  = 1E5 * ones(1, 3);         % Initial error 
    params.motionNoise           = [25, 10, 1];              % Motion noise
    params.measurementNoise      = 25;                       
    params.segmentationThreshold = 0.05;                     
end