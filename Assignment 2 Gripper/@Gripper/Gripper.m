classdef Gripper < RobotBaseClass

    properties(Access = public)
        plyFileNameStem = 'Gripper';
    end

    methods
        %% Constructor
        function self = Gripper(baseTr)
            if nargin == 0
                baseTr = transl(0,0,0);
            end
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end

        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0,'a',0.1005,'alpha',0,'offset',0,'qlim', [pi/3 pi/2]);
            link(2) = Link('d',0,'a',0.0455,'alpha',0,'offset',0,'qlim', [0 pi/6]);

            self.model = SerialLink(link,'name',self.name);
        end
    end
end
