classdef CanHelperTurtleBotVisualizer < handle
    %ExampleHelperTurtleBotVisualizer1 - Class for plotting TurtleBot and local environment
    %   OBJ = ExampleHelperTurtleBotVisualizer(RANGES) creates a plot of the TurtleBot's world and
    %   according to the input axis ranges. The object keeps track of goal
    %   points, pose history, obstacle location data, and handles to graphical
    %   objects.
    %
    %   ExampleHelperTurtleBotVisualizer methods:
    %      plotGoal                 - Plots the goal point on the graph
    %      plotPose                 - Plots the current Turtlebot pose
    %      plotData                 - Plots object location data and robot pose
    %
    %   ExampleHelperTurtleBotVisualizer properties:
    %      FigureHandle             - Handle to the object figure
    %      AxesHandle               - Handle to the object axes
    %      Goal                     - Matrix of goal points
    %      PoseHandle               - Handle to the current pose object
    %      ArrowHandle              - Handle to current direction object
    %      PoseHistory              - Complete pose history matrix
    %      DataHistory              - Data of all occupied points

        
    properties
        FigureHandle = [];      % Handle to the object figure
        AxesHandle = [];        % Handle to the object axes
        Goal = [];              % Matrix of goal points
        PoseHandle = [];        % Handle to the current pose object
        ArrowHandle = [];       % Handle to current direction object
        PoseHistory = [];       % History of all currently plotted robot poses
        DataHistory = [];       % History of all currently plotted laser scans
    end
    
    properties (Constant, Access = private)
        %MaxPoseHistorySize The maximum number of poses that should be stored in PoseHistory
        %   PoseHistory is a FIFO and if it reaches MaxPoseHistorySize, the
        %   oldest element will be deleted.
        MaxPoseHistorySize = 100    
        
        %MaxDataHistorySize The maximum number of laser readings that should be stored in DataHistory
        %   DataHistory is a FIFO and if it reaches MaxDataHistorySize, the
        %   oldest element will be deleted.
        MaxDataHistorySize = 15   
    end
    
    methods (Access = public)
        function obj = CanHelperTurtleBotVisualizer(ranges)
            %ExampleHelperTurtleBotVisualizer - Constructor sets all the figure properties
            obj.FigureHandle = figure('Name','Robot Position','CloseRequestFcn',@obj.closeFigure);
            obj.AxesHandle = axes('Parent',obj.FigureHandle,'XGrid','on','YGrid','on','XLimMode','manual','YLimMode','manual');
            axis(obj.AxesHandle,ranges);
            hold(obj.AxesHandle,'on');
        end
        
        function plot_Goal(obj,goal)
            %PLOTGOAL - Plots the goal point on the graph
            
            if ~ishandle(obj.FigureHandle)
                % Check that figure hasn't been closed
                error('Figure close request: Exiting');
            end
            
            
            plot(obj.AxesHandle,goal(1),goal(2),'d','Color','m','MarkerSize',6);
            obj.Goal = [obj.Goal;goal];
        end
        
        function plot_Orta(obj,a,b)
            %PLOTGOAL - Plots the goal point on the graph
            
            if ~ishandle(obj.FigureHandle)
                % Check that figure hasn't been closed
                error('Figure close request: Exiting');
            end
            
            
            plot(obj.AxesHandle,a,b,'*','Color','r','MarkerSize',6);
%             obj.Goal = [obj.Goal;goal];
        end
        
        function plot_Pose(obj,pose)
            %PLOTPOSE - Plots the current Turtlebot pose
            
            if ~ishandle(obj.FigureHandle)
                % Check that figure hasn't been closed
                error('Figure close request: Exiting');
            end
            
            % Delete current pose objects from plot
%             delete(obj.PoseHandle);
%             delete(obj.ArrowHandle);
            
            % Plot current pose and direction
            obj.PoseHandle = plot(obj.AxesHandle, pose(1),pose(2),'*','Color','g','MarkerSize',5);
% %             obj.ArrowHandle = plot(obj.AxesHandle,[pose(1), pose(1) + 0.5*cos(pose(3))], ...
%                 [pose(2), pose(2) + 0.5*sin(pose(3))], ...
%                 '*','MarkerSize',2,'Color','r','LineStyle','-');

            % Keep a history of previous robot positions (fixed-size FIFO)
            poseHistoryHandle = plot(obj.AxesHandle,pose(1),pose(2),'*','Color','c','MarkerSize',2);
            addNewPoseHandle(obj, poseHistoryHandle);
        end
        
        function plot_Data(obj,pose,data,a,b)
            %PLOTDATA - Plots object location data and robot pose
            
            if ~ishandle(obj.FigureHandle)
                % Check that figure hasn't been closed
                error('Figure close request: Exiting');
            end
            
            % Plot pose
            plot_Pose(obj,pose);
            
            plot_Orta(obj,a,b);
            
            th = pose(3)-pi/2;
            % Compute the world-frame location of laser points
            dataWorld = data*[cos(th) sin(th);-sin(th) cos(th)] ...
                + repmat(pose(1:2),[numel(data(:,1)),1]);
            
            % Plot the transformed laser data on the world map
            % Also keep a history of previous laser data handles (fixed-size FIFO)
            laserDataHandle = plot(obj.AxesHandle,dataWorld(:,1), dataWorld(:,2), '*', 'MarkerSize',1,'Color','k');
%             addNewLaserDataHandle(obj, laserDataHandle);
        end
    end
    
    methods (Access = protected)
        function addNewPoseHandle(obj, poseHandle)
            %addNewPoseHandle Store a new pose graphics handle
            
            obj.PoseHistory = [obj.PoseHistory; poseHandle];
            if length(obj.PoseHistory) > obj.MaxPoseHistorySize
                % If we reached the maximum size of the array, delete the
                % oldest graphics handle.
                delete(obj.PoseHistory(1));
                obj.PoseHistory(1) = [];
            end
        end
        
        function addNewLaserDataHandle(obj, laserDataHandle)
            %addNewLaserDataHandle Store a new laser data graphics handle
            
            obj.DataHistory = [obj.DataHistory; laserDataHandle];
            if length(obj.DataHistory) > obj.MaxDataHistorySize
                % If we reached the maximum size of the array, delete the
                % oldest graphics handle.
                delete(obj.DataHistory(1));
                obj.DataHistory(1) = [];
            end
        end
        
        function closeFigure(obj,~,~)
            %CLOSEFIGURE - Callback function that deletes the figure
            % handle
            
            delete(obj.FigureHandle);
        end
    end
    
end

