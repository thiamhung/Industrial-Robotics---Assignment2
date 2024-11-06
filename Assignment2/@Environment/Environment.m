classdef Environment
    properties(Access = public)              
        % plyFileNameStem = 'Environment';
      
    end
    
    methods
        %% Constructor to initialize the environment
        function self = Environment()
            
            self.Objects(); % Create the fence
        end
        
        
        %% Method to load and position the objects
        function Objects(~)
            hold on
            axis equal
            axis on
           
            PlaceObject('Restaurant.ply', [0, 0, 0]);
            PlaceObject('Chairforbase.PLY', [-2.5, 1, 0]);
            PlaceObject('chair.PLY',[1,2.5,0]);
            PlaceObject('chair.PLY',[1,1.5,0]);
            PlaceObject('chair.PLY',[1,0.5,0]);
            PlaceObject('chair.PLY',[1,-0.5,0]);
            PlaceObject('Grill.ply',[-2.5,2.5,1])
            surf([-5,-5;5,5], [-5,5;-5,5], [0,0;0,0], 'CData', imread('woodGround.jpg'), 'FaceColor', 'texturemap');
            PlaceObject('fireExtinguisher.ply', [3.8,2.8,0.05]);
            PlaceObject('emergencyStopButton.ply', [4.8,3,0]);
            PlaceObject('fireExtinguisher.ply', [-3.8,-2.8,0.05]);
            PlaceObject('emergencyStopButton.ply', [-4.8,-3,0]);
            PlaceObject("personMaleCasual.ply",[-4.8,-2,0]);
            PlaceObject("Stand.ply",[-1,2,1]);
            PlaceObject("Stand.ply",[-1,2.5,1]);

            % % Plates
            % PlaceObject("Plate Square.PLY",[-1, 2-0.15, 1+0.025+0.2]);
            % PlaceObject("Plate Square.PLY",[-1, 2.5-0.15, 1+0.025+0.2]);
            % 
            % % Burgers
            % PlaceObject("hamburger.ply",[-2.3, 2.5, 1.2]);
            % PlaceObject("hamburger.ply",[-2.6, 2.5, 1.2]);

            % Place the object
            hObject = PlaceObject('Shield.ply', [-1.1, -1.4, 1]);

           % Set the transparency (0 is fully transparent, 1 is fully opaque)
            set(hObject, 'FaceAlpha', 0.5); % Set transparency to 50%

        end    
    end
end