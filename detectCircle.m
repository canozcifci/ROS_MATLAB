function [centerX,centerY,blobSize,isObjectDetected] = detectCircle(img,scale)
% Detects circle location and size in image

    % Initialize variables
    centerX = 0;
    centerY = 0;
    blobSize = 0;
    
    % Resize the image
    img = imresize(img,scale,'Antialiasing',false);
    
    % Create mask based on chosen histogram thresholds
    % We convert to L*a*b* space and then
    % Thresholds for BLUE 
    LMin = 0;
    LMax = 70;
    aMin = -100;
    aMax = 100;
    bMin = -100;
    bMax = -15;
    labImg = rgb2lab(img);
    imgBW = (labImg(:,:,1) >= LMin)&(labImg(:,:,1) <= LMax)& ...
            (labImg(:,:,2) >= aMin)&(labImg(:,:,2) <= aMax)& ...
            (labImg(:,:,3) >= bMin)&(labImg(:,:,3) <= bMax);

%%         
    % Detect blobs
    persistent detector
    if isempty(detector)
        detector = vision.BlobAnalysis( ...
                    'BoundingBoxOutputPort',false, ...
                    'AreaOutputPort',false, ...
                    'MajorAxisLengthOutputPort', true, ...
                    'MinimumBlobArea',300, ...
                    'MaximumCount', 10);
    end
    [centroids,majorAxes] = detector(imgBW);
    
    % Estimate the blob location and size, if any are large enough
    if ~isempty(majorAxes)
        
        isObjectDetected = true;
        
        % Find max blob major axis
        [blobSize,maxIdx] = max(majorAxes);
        blobSize = double(blobSize(1));
        
        % Find location of largest blob
        maxLoc = centroids(maxIdx,:);
        centerX = double(maxLoc(1));
        centerY = double(maxLoc(2));
    else
        isObjectDetected = false;
        
    end
    
    % Rescale outputs
    centerX = centerX/scale;
    centerY = centerY/scale;
    blobSize = blobSize/scale;
        
end

