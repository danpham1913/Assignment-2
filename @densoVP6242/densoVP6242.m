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
            link(1) = Link('d',0.28,'a',0,'alpha',pi/2,'offset',0, 'qlim',[-pi pi]) ;
            link(2) = Link('d',0,'a',0.210,'alpha',0,'offset',pi/2, 'qlim',[-pi pi]) ;
            link(3) = Link('d',0,'a',0.233,'alpha',0,'offset',-deg2rad(70.33), 'qlim',[-pi pi]) ;
            link(4) = Link('d',0,'a',0.07,'alpha',pi/2,'offset',-deg2rad(90-70.33), 'qlim',[-pi pi]);       
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end