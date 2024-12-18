classdef LinearUR3 < RobotBaseClass
    %% LinearUR3 UR3 on a non-standard linear rail created by a student

    properties(Access = public)
        plyFileNameStem = 'LinearUR3';  
        gripperL;
        gripperR;
        
        % bricks;
        % Brick_start;
        % Brick_end;
        % v;
        % fence;
        % a;
    end
    methods
        %% Constructor
        function self = LinearUR3(baseTr)

            if nargin < 1
                baseTr = eye(4);
            end
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr*transl(-0.45,-0.5,1)*trotx(pi/2);
            % disp(self.model.base)
            self.PlotAndColourRobot();  
            self.CreateGripper(); % Create the gripper
            self.AttachGripper(); % Attach the gripper to the self
            % self.model.teach(self.model.getpos)
            % self.PlaceBricks();
            % self.PlaceFenceandGround();
            % self.BuildWall();
            % self.Calculation();
            % self.PointCloud();
        end
teach()
        %% Create the self model
        function CreateModel(self)
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            link(2) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            % Incorporate joint limits
            link(1).qlim = [-3.1 0];
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-90 90]*pi/180;
            link(4).qlim = [-90 90]*pi/180;
            link(5).qlim = [-90 90]*pi/180;
            link(6).qlim = [-90 90]*pi/180;
            link(7).qlim = [-180 180]*pi/180;

            link(3).offset = -pi/2;
            link(5).offset = -pi/2;
            self.model = SerialLink(link,'name',self.name);
            hold on
        end


        %% Create the gripper model
        function CreateGripper(self)
            hold on
            % Define the gripper links with the correct parameters
            LINK(1) = Link('d',0, 'a',0.05, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',-deg2rad(16));
            LINK(2) = Link('d',0, 'a',0.045, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',deg2rad(58));
            LINK(3) = Link('d',0, 'a',0.045, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',deg2rad(48));

            gripperBase = eye(4);

            % Create left and right gripper models
            self.gripperL = SerialLink(LINK,'name','GripperL','base',gripperBase);
            self.gripperR = SerialLink(LINK,'name','GripperR','base',gripperBase);

        end
        %% Attach the gripper to the self
        function AttachGripper(self)
          hold on
            % Attach the left gripper
            self.gripperL.base = self.model.fkine(self.model.getpos).T * trotx(pi/2);
            % disp(self.gripperL.base)
            % Attach the right gripper
            self.gripperR.base = self.model.fkine(self.model.getpos).T * trotz(pi)*trotx(pi/2);
             % disp(self.gripperR.base)
            % Plot and colour the grippers
            self.PlotAndColourGripper(self.gripperL);
            self.PlotAndColourGripper(self.gripperR);
        end
        %% Plot and colour the gripper
        function PlotAndColourGripper(self, gripper)
            hold on
            % Load and apply textures for the gripper
            for linkIndex = 0:gripper.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                gripper.faces{linkIndex + 1} = faceData;
                gripper.points{linkIndex + 1} = vertexData;
            end

            gripper.plot3d(zeros(1,gripper.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            gripper.delay = 0;

            % Apply colors
            for linkIndex = 0:gripper.n
                handles = findobj('Tag', gripper.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end    
% %% Place 3 bunches of bricks around the self (new way)
% function PlaceBricks(self)
%     hold on;
% 
%     % Load brick model to get the vertices and face data
%     [f, self.a, data] = plyread('HalfSizedRedGreenBrick.ply', 'tri');
%     self.v = size(self.a, 1);  % Number of vertices
% 
% 
%     % Define the brick positions
%     BrickLocation_1 = [-0.1 0.32 0.034; 0 0.3 0; -0.1 0.3 0;...
%                        -0.2 0.3 0; -0.3 0.3 0; -0.4 0.3 0;...
%                        -0.5 0.3 0; -0.6 0.3 0.034; -0.6 0.3 0];
%     BrickLocation_2 = [-0.3 -0.35 0; -0.35 -0.35 0; -0.40 -0.35 0;...
%                        -0.3 -0.35 0.034; -0.35 -0.35 0.034; -0.40 -0.35 0.034;...
%                        -0.3 -0.35 0.068; -0.35 -0.35 0.068; -0.40 -0.35 0.068];
% 
%     numBricks = size(BrickLocation_1, 1);
%     self.bricks = gobjects(1, numBricks);  % Pre-allocate handles for bricks
% 
%     % Loop to place each brick
%     for i = 1:numBricks
%         % Place the brick using PlaceObject (for visual positioning)
%         self.bricks(i) = PlaceObject('HalfSizedRedGreenBrick.ply', BrickLocation_1(i, :));
% 
%         % Store the starting and ending transformation matrices for the bricks
%         self.Brick_start(:,:,i) = transl(BrickLocation_1(i, 1), BrickLocation_1(i, 2), BrickLocation_1(i, 3) +0.034) * troty(pi);
% 
%         self.Brick_end(:,:,i) = transl(BrickLocation_2(i, 1), BrickLocation_2(i, 2), BrickLocation_2(i, 3) + 0.034) * troty(pi);
%     end 
% end
% 
% 
% %% Safe Working Environment
% function PlaceFenceandGround(self)
% 
%     axis equal
%     axis on
%     hold on
% 
%     % Place the fence object at the origin (or adjust as needed)
%     PlaceObject('fenceAssemblyGreenRectangle4x8x2.5m.ply', [0, 1, -1]);
% 
%     % Place the ground
%     % Load the image for the ground texture
%     img = imread('concrete.jpg');
% 
%     % Define the ground area
%     x = linspace(-4, 4, size(img, 2));
%     y = linspace(-1.75,2, size(img, 1));
% 
%     % Create a meshgrid for the surface
%     [X, Y] = meshgrid(x, y);
%     Z = zeros(size(X)); % Ground is at z=0
% 
%     % Plot the textured ground
%     hold on; % Keep the fence plot
%     surf(X, Y, Z, 'CData', img, 'FaceColor', 'texturemap', 'EdgeColor', 'none');
% 
%     % Set axis properties
%     axis equal;
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     title('Full Model Linear UR3 in safe working environment');
% 
%     % Update the view
%     camlight;
%     lighting gouraud;
% 
%     % Place additional objects in the environment
%     PlaceObject('fireExtinguisher.ply', [2, 2.5, 0]); % Fire extinguisher at [2, 0, 0]
%     PlaceObject('emergencyStopButton.ply', [1, 2.5, 0]); % Emergency stop button at [1, 0, 0]
%     PlaceObject('personMaleOld.ply',[1,3,0]);
% end
% 
% %% Build Wall 3x3
% function BuildWall(self)
%     hold on
% 
%     % Gripper animation ( close and open)
%     open = zeros(1,3);
%     close = [-0.2513    0.6912   -0.4398]; % gripper close joint state
%     close_gripper = jtraj(open, close, 100);
%     open_gripper = jtraj(close, open, 100);
% 
%     % LET build the wall
%     display(['Start Build the 3x3 Wall']);
%     c = zeros(1,7);
%     % Get current joint sate of the UR3e 
%     currentState = self.model.getpos;
%     disp(currentState)
%     for j = 1:9
%         display(['Moving Brick ', num2str(j)]);
%         % Move the robot to brick initial position
%         b = self.model.ikcon(self.Brick_start(:,:,j), currentState); % Calculation Joint state for brick start
%         qMatrix = jtraj(c, b, 100);
%         c = self.model.ikcon(self.Brick_end(:,:,j), currentState); % Calculation joint state for brick end
%         qMatrix2 = jtraj(b, c, 100);
%         % Moving to brick initial position
%         display(['Moving End Effector of  to Brick ', num2str(j), ' initial position']);
%         for i = 1:100
%             pause(0.01);
% 
%             self.model.animate(qMatrix(i, :));
%             self.gripperL.base = self.model.fkine(self.model.getpos).T * trotx(pi/2);
%             self.gripperR.base = self.model.fkine(self.model.getpos).T * trotz(pi) * trotx(pi/2);
%             self.gripperL.animate(self.gripperL.getpos());
%             self.gripperR.animate(self.gripperR.getpos());
%         end
% 
%         % Picking up the brick
%         display(['Picking Up Brick ', num2str(j)]);
%         for i = 1:100
%             pause(0.01);
%             self.gripperL.animate(close_gripper(i, :));
%             self.gripperR.animate(close_gripper(i, :));
% 
%         end
% 
%         % Attach the brick to the end-effector
%         EndEffector = self.model.fkine(self.model.getpos).T; % Get current end-effector transformation matrix
%         updatedPoints = [EndEffector * [self.a, ones(self.v, 1)]']';  % Apply transformation to the brick's vertices
%         self.bricks(j).Vertices = updatedPoints(:, 1:3);% Update the brick's position
% 
%         % Moving to brick final position
%         display(['Moving Brick ', num2str(j), ' to final position']);
%         for i = 1:100
%             pause(0.01);
%             % Move UR3 (without gripper) to initial of the brick
%             self.model.animate(qMatrix2(i, :));
%             %
%             self.gripperL.base = self.model.fkine(self.model.getpos).T * trotx(pi/2);
%             self.gripperR.base = self.model.fkine(self.model.getpos).T * trotz(pi) * trotx(pi/2);
%             self.gripperL.animate(self.gripperL.getpos());
%             self.gripperR.animate(self.gripperR.getpos());
%              % Attach the brick to the end-effector
%             EndEffector = self.model.fkine(self.model.getpos).T;
%             updatedPoints = [EndEffector * [self.a, ones(self.v, 1)]']';
%             self.bricks(j).Vertices = updatedPoints(:, 1:3);
%         end
% 
%         % Dropping off the brick
%         display(['Dropping off Brick ', num2str(j)]);
%         for i = 1:100
%             pause(0.01);
%             self.gripperL.animate(open_gripper(i, :));
%             self.gripperR.animate(open_gripper(i, :));
%         end
%         updatedPoints = [self.Brick_end(:,:,j) * [self.a, ones(self.v, 1)]']';
%         self.bricks(j).Vertices = updatedPoints(:, 1:3);   
%     end
% 
%     display(['Building Wall - Complete']);
% end
% %% Given an end-effector pose, determine the joint state and move the ROBOTT to the required join state
% % function Calculation(self)
% %  desiredPose = transl(0.5, 0.2, 0.3) * trotz(pi/4);
% % currentJointState= self.model.getpos();
% % disp(currentJointState)
% % 
% % %move the self to the desiredPose
% % jointState = self.model.ikine(desiredPose, currentJointState, [1,1,1,0,0,0]);
% % disp('Computed Joint State:');
% % disp(jointState);
% % % Define the target joint angles
% % targetJointAngles = jointState;
% % 
% % % Generate joint trajectories for smooth movement
% % 
% % qMatrix = jtraj(currentJointState, targetJointAngles, 100); % 100 points for smooth movement
% % 
% % % Move the self to the target joint angles
% % for i = 1:100
% %     pause(0.05); % Pause to simulate real-time movement
% %     self.model.animate(qMatrix(i, :)); % Animate the robot's movement
% %     self.gripperL.base = self.model.fkine(self.model.getpos).T * trotx(pi/2);
% %     self.gripperR.base = self.model.fkine(self.model.getpos).T * trotz(pi) * trotx(pi/2);
% %     self.gripperL.animate(self.gripperL.getpos());
% %     self.gripperR.animate(self.gripperR.getpos());
% % end
% % end
% %% Calculate and plot workspace radius & volume ( same with LAB3 SOLUTION)
% function PointCloud(self)
% stepRads = deg2rad(50);
% qlim = self.model.qlim;
% rad2linstep = abs(qlim(1,1))/((360*2)/rad2deg(stepRads)); %change rad to linear steps
% 
% pointCloudeSize = prod(floor((qlim(1:7,2)-qlim(1:7,1))/stepRads + 1));
% pointCloud = zeros(pointCloudeSize,3);
% counter = 1;
% tic
% 
% % No need to worry for q7 because q7 doesn't affect workspace volume
% for q1 = qlim(1,1):rad2linstep:qlim(1,2)
%     for q2 = qlim(2,1):stepRads:qlim(2,2)
%         for q3 = qlim(3,1):stepRads:qlim(3,2)
%             for q4 = qlim(4,1):stepRads:qlim(4,2)
%                 for q5 = qlim(5,1):stepRads:qlim(5,2)
%                     for q6 = qlim(6,1):stepRads:qlim(6,2)
%                         q7=0;
%                         q = [q1,q2,q3,q4,q5,q6,q7];
%                         tr = self.model.fkineUTS(q); 
%                         pointCloud(counter,:) = tr(1:3,4)'; 
%                         counter = counter + 1;
%                         if mod(counter/pointCloudeSize * 100,1) == 0
%                             display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
%                         end
%                     end
%                 end
%             end
%         end
%     end
% end
% % Create a 3D model showing where the end effector can be over all these samples.
% plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
% display(['Calculation is Done, Look at the figure to see the change']);
% end
end
end