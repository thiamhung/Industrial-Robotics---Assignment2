classdef RestaurantSimulation < handle
    %%
    properties(Access = public)
        LinearUR3  % First robot instance
        SecondRobot % Second robot instance
        Environment
        % Variables for burger
        burger;
        burger_start;
        burger_end;
        v_1;  
        a_1;
        updatedPoints_1;
        EndEffector_1;
        % Variables for plate
        plate;
        plate_start;
        plate_end;
        v_2;
        a_2;
        updatedPoints_2;
        EndEffector_2;
        
       
        
    end
    methods
        %% Constructor
        function self = RestaurantSimulation()
          
           close all
           clf
           clc

         % Constructor: Initialize the restaurant environment and robots
            self.LinearUR3 = LinearUR3();  % Your existing Linear UR3 class
            self.SecondRobot = SecondRobot();  % Your existing second robot class
            self.Environment=Environment();  % Setup the environment (ground, walls, tables, etc.)
            self.BurgerandPlatePosition(); %Setup Burger and Plate position
            self.Pickup_PlaceBurger_SecondRobot();
            self.Pickup_PlacePlateAndBurger_LinearUR3e();
            self.MoveSecondRobotWithCollisionDetection();
        end
        %% Place Burger
        function BurgerandPlatePosition(self)
            %     hold on;

    % Load brick model to get the vertices and face data
    [f, self.a_1, data] = plyread('hamburger.ply', 'tri');
    self.v_1 = size(self.a_1, 1);  % Number of vertices
    
    [f_2,self.a_2,data_2]=plyread('Plate Square.PLY', 'tri');
    self.v_2 = size(self.a_2, 1);  % Number of vertices

    % Define the burger positions
    BurgerLocation_1 = [-2.4, 2.5, 1.2;-2.5, 2.5, 1.2];
    BurgerLocation_2 = [-1, 2, 1.1+0.2;-1, 2.5, 1.1+0.2];
    
    % Define the Plate Location
    PlateLocation_1=[ -1, 2-0.15, 1+0.025+0.2 ; -1, 2.5-0.15, 1+0.025+0.2 ];
    % PlateLocation_2=[ 0.2, 2-0.15, 1+0.025+0.2  ; 0.2, 2.5-0.15, 1+0.025+0.2 ];
    PlateLocation_2=[ 0.2, 2-0.15, 1.025  ; 0.2, 2.5-0.15, 1.025 ];

    numBurger = size(BurgerLocation_1, 1);
    
    self.burger = gobjects(1, numBurger);  % Pre-allocate handles for Burger
    self.plate  = gobjects(1, numBurger);  % Pre-allocate handles for Burger
    % Loop to place each burger
    for i = 1:numBurger

        % Place the Burger using PlaceObject (for visual positioning)
        self.burger(i) = PlaceObject('hamburger.ply', BurgerLocation_1(i, :));
        % Store the starting and ending transformation matrices for the burger
        self.burger_start(:,:,i) = transl(BurgerLocation_1(i, 1), BurgerLocation_1(i, 2), BurgerLocation_1(i, 3))*troty(2*pi/3)*transl(-0.1,0,0);
        self.burger_end(:,:,i) = transl(BurgerLocation_2(i, 1), BurgerLocation_2(i, 2), BurgerLocation_2(i, 3));

        % Place the Plate using PlaceObject
        self.plate(i) = PlaceObject('Plate Square.PLY', PlateLocation_1(i, :));
         % Store the starting and ending transformation matrices for the bricks
         
        self.plate_start(:,:,i) = transl(PlateLocation_1(i, 1), PlateLocation_1(i, 2), PlateLocation_1(i, 3))*troty(pi/2)*trotx(-pi/2);
        self.plate_end(:,:,i) = transl(PlateLocation_2(i, 1), PlateLocation_2(i, 2), PlateLocation_2(i, 3))*troty(pi/2)*trotx(-pi/2);
    end 
  
end

%% Pick up and Place Burger to new position using RMRC
function Pickup_PlaceBurger_SecondRobot(self)

    % Gripper animation (close and open)
    open = zeros(1,3);
    close = [-0.2513, 0.6912, -0.4398]; % Gripper close joint state
    close_gripper = jtraj(open, close, 50);
    open_gripper = jtraj(close, open, 50);
    
    display(['Start pickup burger']);
    
    for j = 1:2
        display([' Moving to burger ', num2str(j)]);
        [qMatrix]=RMRCMove(self.burger_start(:,:,j),self.SecondRobot); % RMRC movement to start position
        for i = 1:size(qMatrix, 1)
            self.SecondRobot.model.animate(qMatrix(i, :));
            self.SecondRobot.GRIPPERL.base = self.SecondRobot.model.fkine(qMatrix(i, :)).T * trotx(pi/2) * troty(pi/2);
            self.SecondRobot.GRIPPERR.base = self.SecondRobot.model.fkine(qMatrix(i, :)).T * trotz(pi) * trotx(pi/2) * troty(pi/2);
            self.SecondRobot.GRIPPERL.animate(self.SecondRobot.GRIPPERL.getpos());
            self.SecondRobot.GRIPPERR.animate(self.SecondRobot.GRIPPERR.getpos());
            pause(0.01);  % Adjust pause as needed
        end
        % Picking up the burger
        display(['Picking up Burger ', num2str(j)]);
        for i = 1:50
            pause(0.01);
            self.SecondRobot.GRIPPERL.animate(close_gripper(i, :));
            self.SecondRobot.GRIPPERR.animate(close_gripper(i, :));
             
        end
        % Attach the burger to the end-effector
        self.EndEffector_1 = self.SecondRobot.model.fkine(self.SecondRobot.model.getpos).T; % Get current end-effector transformation matrix
        self.updatedPoints_1 = [self.EndEffector_1 * troty(2*pi/3)*transl(-0.1,0,0.0)*[self.a_1, ones(self.v_1, 1)]']';  % Apply transformation to the burger's vertices
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3); % Update the burger's position
        
       

        %NMoving to burger drop-off position
        [qMatrix]=RMRCMove(self.burger_end(:,:,j),self.SecondRobot); % RMRC movement to end position
        
         for i = 1:size(qMatrix, 1)
            self.SecondRobot.model.animate(qMatrix(i, :));
            self.SecondRobot.GRIPPERL.base = self.SecondRobot.model.fkine(qMatrix(i, :)).T * trotx(pi/2) * troty(pi/2);
            self.SecondRobot.GRIPPERR.base = self.SecondRobot.model.fkine(qMatrix(i, :)).T * trotz(pi) * trotx(pi/2) * troty(pi/2);
            self.SecondRobot.GRIPPERL.animate(self.SecondRobot.GRIPPERL.getpos());
            self.SecondRobot.GRIPPERR.animate(self.SecondRobot.GRIPPERR.getpos());
            pause(0.01);  % Adjust pause as needed

             % Attach the burger to the end-effector
       self.EndEffector_1 = self.SecondRobot.model.fkine(self.SecondRobot.model.getpos).T; % Get current end-effector transformation matrix
        self.updatedPoints_1 = [self.EndEffector_1 * troty(2*pi/3)*transl(-0.1,0,0.0)*[self.a_1, ones(self.v_1, 1)]']';  % Apply transformation to the burger's vertices
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3); % Update the burger's position
        end
        display(['Dropping off Burger ', num2str(j)]);

        for i = 1:50
            pause(0.01);
            self.SecondRobot.GRIPPERL.animate(open_gripper(i, :));
            self.SecondRobot.GRIPPERR.animate(open_gripper(i, :));
            
        end
       
        % Update burger position after drop-off
        self.updatedPoints_1 = [self.burger_end(:,:,j) * [self.a_1, ones(self.v_1, 1)]']';
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3);

        steps=50;
       % Return to home position [0, 0, 0, 0, 0, 0]
    homeQ = [0, 0, 0, 0, 0, 0];  % Home position joint configuration
    currentQ = self.SecondRobot.model.getpos();  % Get current joint state
    returnTrajectory = jtraj(currentQ, homeQ, steps);  % Create joint trajectory to home
    
    for i = 1:steps
        self.SecondRobot.model.animate(returnTrajectory(i, :));  % Animate robot back to home
        self.SecondRobot.GRIPPERL.base = self.SecondRobot.model.fkine(returnTrajectory(i, :)).T * trotx(pi/2) * troty(pi/2);
        self.SecondRobot.GRIPPERR.base = self.SecondRobot.model.fkine(returnTrajectory(i, :)).T * trotz(pi) * trotx(pi/2) * troty(pi/2);
        self.SecondRobot.GRIPPERL.animate(self.SecondRobot.GRIPPERL.getpos());
        self.SecondRobot.GRIPPERR.animate(self.SecondRobot.GRIPPERR.getpos());
        pause(0.01);
    end
    disp('Robot has returned to home position.');
    end
    
end



%% Pick up and Deliver the Plate with Burger to customer
function Pickup_PlacePlateAndBurger_LinearUR3e(self)
    
    % Gripper animation ( close and open)
    open = zeros(1,3);
    close = [-0.2513    0.6912   -0.4398]; % gripper close joint state
    close_gripper = jtraj(open, close, 100);
    open_gripper = jtraj(close, open, 100);

    display(['Start pick up the plates']);

    %First movement, go straight along the linear
    % [qMatrix]=RMRCMove(transl(-0.45, 1.786, 1.694),self.LinearUR3);
    currentState= self.LinearUR3.model.getpos;
    q= self.LinearUR3.model.ikcon(transl(-0.45, 1.786, 1.694),currentState);
    qMatrix= jtraj(currentState,q,50);
    disp(q);
    
    for i=1:size(qMatrix,1)
         self.LinearUR3.model.animate(qMatrix(i, :));
         self.LinearUR3.gripperL.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotx(pi/2);
         self.LinearUR3.gripperR.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotz(pi) * trotx(pi/2);
         self.LinearUR3.gripperL.animate(self.LinearUR3.gripperL.getpos());
         self.LinearUR3.gripperR.animate(self.LinearUR3.gripperR.getpos());
         pause(0.01)
    end
    
    %Second movement, the robot heads to pick up position
    for j = 1:2
        display(['Robot is Moving Plate ', num2str(j)]);
        % Move the robot to plate initial position
        % Get current joint sate of the UR3e 
        currentState = self.LinearUR3.model.getpos;
        q_0=jtraj(currentState,[ -2.0913         0   -1.1594   -0.4710         0         0    1.5708],100);
        
        % Moving to Plate initial position
        display(['Moving End Effector of  to Plate ', num2str(j), ' initial position']);
        for i = 1:100
            pause(0.01);
            self.LinearUR3.model.animate(q_0(i, :));
            self.LinearUR3.gripperL.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotx(pi/2);
            self.LinearUR3.gripperR.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotz(pi) * trotx(pi/2);
            self.LinearUR3.gripperL.animate(self.LinearUR3.gripperL.getpos());
            self.LinearUR3.gripperR.animate(self.LinearUR3.gripperR.getpos());
        end
        currentState_1 = self.LinearUR3.model.getpos;
        a = self.LinearUR3.model.ikcon(self.plate_start(:,:,j), currentState_1); % Calculation Joint state for brick start
        q_1 = jtraj(currentState_1, a, 100);
        for i = 1:100
            pause(0.01);
            self.LinearUR3.model.animate(q_1(i, :));
            self.LinearUR3.gripperL.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotx(pi/2);
            self.LinearUR3.gripperR.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotz(pi) * trotx(pi/2);
            self.LinearUR3.gripperL.animate(self.LinearUR3.gripperL.getpos());
            self.LinearUR3.gripperR.animate(self.LinearUR3.gripperR.getpos());
        end
        % Picking up the Plate with attached Burger
        display(['Picking Up Plate ', num2str(j)]);
        for i = 1:100
            pause(0.01);
            self.LinearUR3.gripperL.animate(close_gripper(i, :));
            self.LinearUR3.gripperR.animate(close_gripper(i, :));

        end
        EndEffector = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T; % Get current end-effector transformation matrix
        updatedPoints = [EndEffector*trotx(pi/2)*troty(-pi/2)*[self.a_2, ones(self.v_2, 1)]']';  % Apply transformation to the Plate's vertices
        self.plate(j).Vertices = updatedPoints(:, 1:3);% Update the plate's position
      
         % Apply the same transform to the burger vertices
        self.updatedPoints_1 = [EndEffector * trotx(pi/2)*troty(-pi/2)*transl(0.025,0.15,0.1)*[self.a_1, ones(self.v_1, 1)]']';  % Apply transformation to burger vertices
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3);



        currentState_2= self.LinearUR3.model.getpos;
        q_2=jtraj(currentState_2,[-2.1619    0.0499         0   -1.5708    0.0005   -0.0015    3.0089],100);
        for i= 1:100
            pause(0.01);
            self.LinearUR3.model.animate(q_2(i, :));
            self.LinearUR3.gripperL.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotx(pi/2);
            self.LinearUR3.gripperR.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotz(pi) * trotx(pi/2);
            self.LinearUR3.gripperL.animate(self.LinearUR3.gripperL.getpos());
            self.LinearUR3.gripperR.animate(self.LinearUR3.gripperR.getpos());
            EndEffector = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T; % Get current end-effector transformation matrix
            updatedPoints = [EndEffector*trotx(pi/2)*troty(-pi/2)*[self.a_2, ones(self.v_2, 1)]']';  % Apply transformation to the Plate's vertices
            self.plate(j).Vertices = updatedPoints(:, 1:3);% Update the plate's position
           

             % Apply the same transform to the burger vertices
        self.updatedPoints_1 = [EndEffector * trotx(pi/2)*troty(-pi/2)*transl(0.025,0.15,0.1)*[self.a_1, ones(self.v_1, 1)]']';  % Apply transformation to burger vertices
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3);





        end
        
        
        currentState_3=self.LinearUR3.model.getpos;
        q_3=jtraj(currentState_3,[-2.1619    0.0499    0.0000         0    0.0005   -0.0015    1.5708],100);
        for i=1:100
            self.LinearUR3.model.animate(q_3(i, :));
            self.LinearUR3.gripperL.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotx(pi/2);
            self.LinearUR3.gripperR.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotz(pi) * trotx(pi/2);
            self.LinearUR3.gripperL.animate(self.LinearUR3.gripperL.getpos());
            self.LinearUR3.gripperR.animate(self.LinearUR3.gripperR.getpos());

            pause(0.01)

            EndEffector = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T; % Get current end-effector transformation matrix
            updatedPoints = [EndEffector*trotx(pi/2)*troty(-pi/2)*[self.a_2, ones(self.v_2, 1)]']';  % Apply transformation to the Plate's vertices
            self.plate(j).Vertices = updatedPoints(:, 1:3);% Update the plate's position
            
            
         % Apply the same transform to the burger vertices
        self.updatedPoints_1 = [EndEffector * trotx(pi/2)*troty(-pi/2)*transl(0.025,0.15,0.1)*[self.a_1, ones(self.v_1, 1)]']';  % Apply transformation to burger vertices
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3);


        end
        
        currentState_4=self.LinearUR3.model.getpos;
        q_4=jtraj(currentState_4,[-2.1619    0.0499    0    1.5708    0.0005   -0.0015    0],100);
         for i=1:100
            self.LinearUR3.model.animate(q_4(i, :));
            self.LinearUR3.gripperL.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotx(pi/2);
            self.LinearUR3.gripperR.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotz(pi) * trotx(pi/2);
            self.LinearUR3.gripperL.animate(self.LinearUR3.gripperL.getpos());
            self.LinearUR3.gripperR.animate(self.LinearUR3.gripperR.getpos());

            pause(0.01)

            EndEffector = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T; % Get current end-effector transformation matrix
            updatedPoints = [EndEffector*trotx(pi/2)*troty(-pi/2)*[self.a_2, ones(self.v_2, 1)]']';  % Apply transformation to the Plate's vertices
            self.plate(j).Vertices = updatedPoints(:, 1:3);% Update the plate's position
            
             % Apply the same transform to the burger vertices
        self.updatedPoints_1 = [EndEffector * trotx(pi/2)*troty(-pi/2)*transl(0.025,0.15,0.1)*[self.a_1, ones(self.v_1, 1)]']';  % Apply transformation to burger vertices
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3);


         end

        currentState_5=self.LinearUR3.model.getpos;
        b = self.LinearUR3.model.ikcon(self.plate_end(:,:,j), currentState_5); % Calculation Joint state for brick start
        q_5 = jtraj(currentState_5, b, 100);
         for i= 1:100
            pause(0.01);
            self.LinearUR3.model.animate(q_5(i, :));
            self.LinearUR3.gripperL.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotx(pi/2);
            self.LinearUR3.gripperR.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotz(pi) * trotx(pi/2);
            self.LinearUR3.gripperL.animate(self.LinearUR3.gripperL.getpos());
            self.LinearUR3.gripperR.animate(self.LinearUR3.gripperR.getpos());
            EndEffector = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T; % Get current end-effector transformation matrix
            updatedPoints = [EndEffector*trotx(pi/2)*troty(-pi/2)*[self.a_2, ones(self.v_2, 1)]']';  % Apply transformation to the Plate's vertices
            self.plate(j).Vertices = updatedPoints(:, 1:3);% Update the plate's position
           
              % Apply the same transform to the burger vertices
        self.updatedPoints_1 = [EndEffector * trotx(pi/2)*troty(-pi/2)*transl(0.025,0.15,0.1)*[self.a_1, ones(self.v_1, 1)]']';  % Apply transformation to burger vertices
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3);



         end
        % Dropping off the brick
        display(['Dropping off Plate ', num2str(j)]);
        for i = 1:100
            pause(0.01);
            self.LinearUR3.gripperL.animate(open_gripper(i, :));
            self.LinearUR3.gripperR.animate(open_gripper(i, :));
        end
        updatedPoints = [self.plate_end(:,:,j)*trotx(pi/2)*troty(-pi/2) * [self.a_2, ones(self.v_2, 1)]']';
        self.plate(j).Vertices = updatedPoints(:, 1:3); 
        self.updatedPoints_1 = [EndEffector * trotx(pi/2)*troty(-pi/2)*transl(0.1,0.13,0.05)*[self.a_1, ones(self.v_1, 1)]']';  % Apply transformation to burger vertices
        self.burger(j).Vertices = self.updatedPoints_1(:, 1:3);



        q_6=jtraj(self.LinearUR3.model.getpos,currentState,100);
        for i= 1:100
            pause(0.01);
            self.LinearUR3.model.animate(q_6(i, :));
            self.LinearUR3.gripperL.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotx(pi/2);
            self.LinearUR3.gripperR.base = self.LinearUR3.model.fkine(self.LinearUR3.model.getpos).T * trotz(pi) * trotx(pi/2);
            self.LinearUR3.gripperL.animate(self.LinearUR3.gripperL.getpos());
            self.LinearUR3.gripperR.animate(self.LinearUR3.gripperR.getpos());
            
        end
      
    end
    % self.LinearUR3.model.teach(self.LinearUR3.model.getpos)
end

%% Function to move SecondRobot with collision detection
        function MoveSecondRobotWithCollisionDetection(self)
            % Initialize robot position
            q = self.SecondRobot.model.getpos();
            
            % Load human.ply model
            hold on
            person = PlaceObject('personMaleConstruction.ply', [-2.5, 0, 0]);
            
            % Generate trajectory qMatrix for the robot
            q_Trans = transl(-2.5, -1, 2);
            q2 = self.SecondRobot.model.ikcon(q_Trans, q);
            q3 = jtraj(q, q2, 100);
            
            % Animate robot with collision detection
            for i = 1:100
                if i <= 100
                    collision = Collision_Dectection(self.SecondRobot, q3(i, :), person);
                    self.SecondRobot.GRIPPERL.base = self.SecondRobot.model.fkine(self.SecondRobot.model.getpos).T *trotx(pi/2)*troty(pi/2);
                    self.SecondRobot.GRIPPERR.base = self.SecondRobot.model.fkine(self.SecondRobot.model.getpos).T *trotz(pi)*trotx(pi/2)*troty(pi/2);
                    self.SecondRobot.GRIPPERL.animate( self.SecondRobot.GRIPPERL.getpos());
                    self.SecondRobot.GRIPPERR.animate(self.SecondRobot.GRIPPERR.getpos());
                    if collision == 1
                        disp('Collision detected 1 step ahead. Stopping animation.');
                        break; % Stop the animation if a collision is detected
                    end
                end
                % Animate the robot step by step
                self.SecondRobot.model.animate(q3(i, :));
                pause(0.01);
            end
    end
end
end
