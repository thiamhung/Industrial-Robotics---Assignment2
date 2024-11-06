classdef SecondRobot < RobotBaseClass
    properties(Access = public)   
        plyFileNameStem = 'SecondRobot';
        GRIPPERL;
        GRIPPERR;
    end
        
    methods 
       % Constructor
function self = SecondRobot(baseTr)
    if nargin < 1
        baseTr = eye(4);
    end
    self.CreateModel();
    
    self.model.base = self.model.base.T * baseTr*transl(-2.5,1.0,0.4)*trotz(pi/2);
    self.PlotAndColourRobot();
    self.CreateGripper(); % Create the gripper
    self.AttachGripper(); % Attach the gripper to the self
    
    % self.model.teach(self.model.getpos);
end  
        %% Create the robot model
        function CreateModel(self)
             link(1) = Link('d',0.245*2.5  ,'a',0.15*2.5,'alpha',-pi/2,  'offset',0);
             link(2) = Link('d',    0      ,'a',0.35*2.5,'alpha',   0 ,  'offset',-pi/2);
             link(3) = Link('d',0.025*2.5  ,'a',0,       'alpha',-pi/2,  'offset',0);
             link(4) = Link('d',0.361*2.5  ,'a',0,       'alpha',-pi/2,  'offset',0);
             link(5)=  Link('d',    0      ,'a',0,       'alpha',pi/2 ,  'offset',0);
             link(6)=  Link('d',(0.098)*2.5,'a',0,       'alpha',   0 ,  'offset',0);
            
            
            link(1).qlim = [-180 60]*pi/180;
            link(2).qlim = [-45 15]*pi/180;
            link(3).qlim = [-45 20]*pi/180;
            link(4).qlim = [-180 180]*pi/180;
            link(5).qlim = [-90 90]*pi/180;
            link(6).qlim = [-180 180]*pi/180;
            self.model = SerialLink(link, 'name', self.name);
            
        end
         %% Create the gripper model
        function CreateGripper(self)

            % Define the gripper links with the correct parameters
            LINK(1) = Link('d',0, 'a',0.05*2.5, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',-deg2rad(16));
            LINK(2) = Link('d',0, 'a',0.045*2.5, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',deg2rad(58));
            LINK(3) = Link('d',0, 'a',0.045*2.5, 'alpha',0, 'qlim',deg2rad([-90 90]), 'offset',deg2rad(48));

            GRIPPERBASE = eye(4);

            % Create left and right gripper models
            self.GRIPPERL = SerialLink(LINK,'name','GRIPPERL','base',GRIPPERBASE);
            self.GRIPPERR = SerialLink(LINK,'name','GRIPPERR','base',GRIPPERBASE);
        end
        %% Attach the gripper to the self
        function AttachGripper(self)
            % Attach the left gripper
            self.GRIPPERL.base = self.model.fkine(self.model.getpos).T*trotx(pi/2)*troty(pi/2);
            % disp(self.GRIPPERL.base)
            % Attach the right gripper
            self.GRIPPERR.base = self.model.fkine(self.model.getpos).T *trotz(pi)*trotx(pi/2)*troty(pi/2);
             % disp(self.GRIPPERR.base)
            % Plot and colour the grippers
            self.PlotAndColourGripper(self.GRIPPERL);
            self.PlotAndColourGripper(self.GRIPPERR);
        end
        %% Plot and colour the gripper
        function PlotAndColourGripper(self, GRIPPER)
            % Load and apply textures for the gripper
            for linkIndex = 0:GRIPPER.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                GRIPPER.faces{linkIndex + 1} = faceData;
                GRIPPER.points{linkIndex + 1} = vertexData;
            end

            GRIPPER.plot3d(zeros(1,GRIPPER.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            GRIPPER.delay = 0;

            % Apply colors
            for linkIndex = 0:GRIPPER.n
                handles = findobj('Tag', GRIPPER.name);
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
        
   

    end
end
