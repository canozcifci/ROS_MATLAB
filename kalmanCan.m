%% Kalman example by Can
function [tracked] = kalmanCan(detected,params,isObjectDetected)

isTrackInitialized = false;

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
            tracked = zeros(1,2);
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
    
end