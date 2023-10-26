classdef densoVP6242 < RobotBaseClass
    %% densoVP6242 on a non-standard linear rail created by a student

    properties(Access = public)              
         plyFileNameStem = 'densoVP6242';
    end
    
    methods
%% Define robot Function 
function self = densoVP6242(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                 self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            hold on
            drawnow
        end
%% Create the robot model
        function CreateModel(self)   
            % Create the densoVP6242 model 
            link(1) = Link('d',0.281,'a',0,'alpha',pi/2,'offset',0, 'qlim',[-pi pi]);
            link(2) = Link('d',0,'a',0.212,'alpha',0,'offset',pi/2, 'qlim',[-pi pi]);
            link(3) = Link('d',0,'a',0,'alpha',pi/2,'offset',pi/2, 'qlim',[-pi pi]);
            link(4) = Link('d',0.075,'a',0.117,'alpha',pi/2,'offset',0, 'qlim',[0 0]);
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'offset',pi/2, 'qlim',[0 0]);
            link(6) = Link('d',0.1,'a',0,'alpha',pi/2,'offset',deg2rad(0), 'qlim',[-pi pi]);
            link(7) = Link('d',0,'a',0,'alpha',-pi/2,'offset',0, 'qlim',[-pi pi]);
            link(8) = Link('d',0.175,'a',0,'alpha',0,'offset',0, 'qlim',[-pi pi]);
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end