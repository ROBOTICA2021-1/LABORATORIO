classdef GUI_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure       matlab.ui.Figure
        GridLayout     matlab.ui.container.GridLayout
        LeftPanel      matlab.ui.container.Panel
        J1Label        matlab.ui.control.Label
        Slider1        matlab.ui.control.Slider
        J2Label        matlab.ui.control.Label
        Slider2        matlab.ui.control.Slider
        J3Label        matlab.ui.control.Label
        Slider3        matlab.ui.control.Slider
        J4Label        matlab.ui.control.Label
        Slider4        matlab.ui.control.Slider
        J5Label        matlab.ui.control.Label
        Slider5        matlab.ui.control.Slider
        J6Label        matlab.ui.control.Label
        Slider6        matlab.ui.control.Slider
        XLabel         matlab.ui.control.Label
        X              matlab.ui.control.Label
        YLabel         matlab.ui.control.Label
        Y              matlab.ui.control.Label
        ZLabel         matlab.ui.control.Label
        Z              matlab.ui.control.Label
        ROLLLabel      matlab.ui.control.Label
        ROLL           matlab.ui.control.Label
        PITCHLabel     matlab.ui.control.Label
        PITCH          matlab.ui.control.Label
        YAWLabel       matlab.ui.control.Label
        YAW            matlab.ui.control.Label
        J1             matlab.ui.control.NumericEditField
        BASELabel      matlab.ui.control.Label
        SHOULDERLabel  matlab.ui.control.Label
        J2             matlab.ui.control.NumericEditField
        ELBOWLabel     matlab.ui.control.Label
        J3             matlab.ui.control.NumericEditField
        WRIST1Label    matlab.ui.control.Label
        J4             matlab.ui.control.NumericEditField
        WRIST2Label    matlab.ui.control.Label
        J5             matlab.ui.control.NumericEditField
        WRIST3Label    matlab.ui.control.Label
        J6             matlab.ui.control.NumericEditField
        ButtonHOME     matlab.ui.control.Button
        TEAM           matlab.ui.control.Label
        AUTOR          matlab.ui.control.Label
        RightPanel     matlab.ui.container.Panel
        UIAxes         matlab.ui.control.UIAxes
        ButtonGroup    matlab.ui.container.ButtonGroup
        DEGButton      matlab.ui.control.RadioButton
        RADButton      matlab.ui.control.RadioButton
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    
    properties (Access = private)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        robot % Universal Robots UR5 construido con RST
        config % Vector de configuraciones de 'robot': [q1 q2 q3 q4 q5 q6]
        configHome
        q1
        q2
        q3
        q4
        q5
        q6
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    methods (Access = private)

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SHOW()            
        function showRobot(app)
            axis(app.UIAxes,'off')
            show(app.robot,app.config,'PreservePlot',false);
            %axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]);
            axis auto
            Frame = getframe(gcf);
            [image, ~] = frame2im(Frame);
            imshow(image,'Parent',app.UIAxes)
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function showCoordinates(app)
            if app.DEGButton.Value==true
                app.J1.Value=rad2deg(app.Slider1.Value); app.J1.ValueDisplayFormat = '%3.2f°';
                app.J2.Value=rad2deg(app.Slider2.Value); app.J2.ValueDisplayFormat = '%3.2f°';
                app.J3.Value=rad2deg(app.Slider3.Value); app.J3.ValueDisplayFormat = '%3.2f°';
                app.J4.Value=rad2deg(app.Slider4.Value); app.J4.ValueDisplayFormat = '%3.2f°';
                app.J5.Value=rad2deg(app.Slider5.Value); app.J5.ValueDisplayFormat = '%3.2f°';
                app.J6.Value=rad2deg(app.Slider6.Value); app.J6.ValueDisplayFormat = '%3.2f°';
            end
            if app.RADButton.Value==true
                app.J1.Value=app.Slider1.Value; app.J1.ValueDisplayFormat = '%3.2f';
                app.J2.Value=app.Slider2.Value; app.J2.ValueDisplayFormat = '%3.2f';
                app.J3.Value=app.Slider3.Value; app.J3.ValueDisplayFormat = '%3.2f';
                app.J4.Value=app.Slider4.Value; app.J4.ValueDisplayFormat = '%3.2f';
                app.J5.Value=app.Slider5.Value; app.J5.ValueDisplayFormat = '%3.2f';
                app.J6.Value=app.Slider6.Value; app.J6.ValueDisplayFormat = '%3.2f';
            end
                
            TCP=getTransform(app.robot,app.config,'endeffector');
            POS=tform2trvec(TCP)*1000;
            ROT=rad2deg(tform2eul(TCP,'XYZ'));
            format compact;
            app.X.Text = string(round(POS(1),2,"decimals"));
            app.Y.Text = string(round(POS(2),2,"decimals"));
            app.Z.Text = string(round(POS(3),2,"decimals"));
            app.ROLL.Text = string(round(ROT(1),2,"decimals"));
            app.PITCH.Text = string(round(ROT(2),2,"decimals"));
            app.YAW.Text = string(round(ROT(3),2,"decimals"));
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Universal Robots UR5 por RST
            app.robot = rigidBodyTree;

            body1 = rigidBody('body1'); 
            jnt1 = rigidBodyJoint('jnt1','revolute'); 
            jnt1.HomePosition = 0; 
            tform = trvec2tform([0,0,89.2/1000])*axang2tform([1 0 0 0]); % User defined 
            setFixedTransform(jnt1,tform); 
            body1.Joint = jnt1; 
            addVisual(body1,"Mesh","ur5/base.stl",trvec2tform([0,0,-89.2/1000]));
            addVisual(body1,"Mesh","ur5/shoulder.stl");
            addBody(app.robot,body1,'base')

            body2 = rigidBody('body2'); 
            jnt2 = rigidBodyJoint('jnt2','revolute'); 
            jnt2.HomePosition = -pi/2; % User defined 
            tform2 = trvec2tform([0,134/1000,0])*axang2tform([1 0 0 -pi/2]); % User defined 
            setFixedTransform(jnt2,tform2); 
            body2.Joint = jnt2;
            addVisual(body2,"Mesh","ur5/upper_arm.stl",axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 pi/2]));
            addBody(app.robot,body2,'body1'); % Add body2 to body1

            body3 = rigidBody('body3'); 
            jnt3 = rigidBodyJoint('jnt3','revolute'); 
            jnt3.HomePosition = 0; % User defined 
            tform3 = trvec2tform([425/1000,0,-119/1000])*axang2tform([1 0 0 pi]); %User defined 
            setFixedTransform(jnt3,tform3); 
            body3.Joint = jnt3; 
            addVisual(body3,"Mesh","ur5/forearm.stl",axang2tform([0 1 0 pi/2])*axang2tform([0 0 1 -pi/2]));
            addBody(app.robot,body3,'body2'); % Add body3 to body2

            body4 = rigidBody('body4'); 
            jnt4 = rigidBodyJoint('jnt4','revolute'); 
            jnt4.HomePosition = pi/2; % User defined 
            tform4 = trvec2tform([392/1000,0,-94.75/1000])*axang2tform([1 0 0 pi]); %User defined 
            setFixedTransform(jnt4,tform4); 
            body4.Joint = jnt4; 
            addVisual(body4,"Mesh","ur5/wrist_1.stl",trvec2tform([0,0,-94.75/1000])*axang2tform([1 0 0 pi/2]));
            addBody(app.robot,body4,'body3'); % Add body4 to body3

            body5 = rigidBody('body5'); 
            jnt5 = rigidBodyJoint('jnt5','revolute'); 
            jnt5.HomePosition = 0; % User defined 
            tform5 = trvec2tform([0,-94.75/1000,0])*axang2tform([1 0 0 pi/2]); %User defined 
            setFixedTransform(jnt5,tform5);
            body5.Joint = jnt5; 
            addVisual(body5,"Mesh","ur5/wrist_2.stl",trvec2tform([0,0,-94.75/1000]));
            addBody(app.robot,body5,'body4'); % Add body5 to body4

            body6 = rigidBody('body6'); 
            jnt6 = rigidBodyJoint('jnt6','revolute'); 
            jnt6.HomePosition = 0; % User defined 
            tform6 = trvec2tform([0,0,0])*axang2tform([1 0 0 -pi/2]); %User defined 
            setFixedTransform(jnt6,tform6);
            body6.Joint = jnt6;
            addVisual(body6,"Mesh","ur5/wrist_3.stl",axang2tform([1 0 0 pi/2]));
            addBody(app.robot,body6,'body5'); % Add body6 to body5

            bodyEndEffector = rigidBody('endeffector'); 
            tform7 = trvec2tform([0, 0, 81.5/1000]); % User defined 
            setFixedTransform(bodyEndEffector.Joint,tform7); 
            addBody(app.robot,bodyEndEffector,'body6');
            
            app.config = homeConfiguration(app.robot);
            app.configHome = homeConfiguration(app.robot); 
            showRobot(app);
            showCoordinates(app);
        end

        % Value changing function: Slider1
        function Slider1ValueChanging(app, event)
            app.q1 = event.Value;
            app.config(1).JointPosition = app.configHome(1).JointPosition + app.q1;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changing function: Slider2
        function Slider2ValueChanging(app, event)
            app.q2 = event.Value;
            app.config(2).JointPosition = app.configHome(2).JointPosition + app.q2;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changing function: Slider3
        function Slider3ValueChanging(app, event)
            app.q3 = event.Value;
            app.config(3).JointPosition = app.configHome(3).JointPosition + app.q3;
            showRobot(app);       
            showCoordinates(app);
        end

        % Value changing function: Slider4
        function Slider4ValueChanging(app, event)
            app.q4 = event.Value;
            app.config(4).JointPosition = app.configHome(4).JointPosition + app.q4;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changing function: Slider5
        function Slider5ValueChanging(app, event)
            app.q5 = event.Value;
            app.config(5).JointPosition = app.configHome(5).JointPosition + app.q5;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changing function: Slider6
        function Slider6ValueChanging(app, event)
            app.q6 = event.Value;
            app.config(6).JointPosition = app.configHome(6).JointPosition + app.q6;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changed function: J1
        function J1ValueChanged(app, event)
            if app.DEGButton.Value==true
                app.Slider1.Value = deg2rad(app.J1.Value);
            end
            if app.RADButton.Value==true
                app.Slider1.Value = app.J1.Value;
            end
            app.config(1).JointPosition = app.configHome(1).JointPosition + app.Slider1.Value;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changed function: J2
        function J2ValueChanged(app, event)
            if app.DEGButton.Value==true
                app.Slider2.Value = deg2rad(app.J2.Value);
            end
            if app.RADButton.Value==true
                app.Slider2.Value = app.J2.Value;
            end
            app.config(2).JointPosition = app.configHome(2).JointPosition + app.Slider2.Value;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changed function: J3
        function J3ValueChanged(app, event)
            if app.DEGButton.Value==true
                app.Slider3.Value = deg2rad(app.J3.Value);
            end
            if app.RADButton.Value==true
                app.Slider3.Value = app.J3.Value;
            end
            app.config(3).JointPosition = app.configHome(3).JointPosition + app.Slider3.Value;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changed function: J4
        function J4ValueChanged(app, event)
            if app.DEGButton.Value==true
                app.Slider4.Value = deg2rad(app.J4.Value);
            end
            if app.RADButton.Value==true
                app.Slider4.Value = app.J4.Value;
            end
            app.config(4).JointPosition = app.configHome(4).JointPosition + app.Slider4.Value;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changed function: J5
        function J5ValueChanged(app, event)
            if app.DEGButton.Value==true
                app.Slider5.Value = deg2rad(app.J5.Value);
            end
            if app.RADButton.Value==true
                app.Slider5.Value = app.J5.Value;
            end
            app.config(5).JointPosition = app.configHome(5).JointPosition + app.Slider5.Value;
            showRobot(app);
            showCoordinates(app);
        end

        % Value changed function: J6
        function J6ValueChanged(app, event)
            if app.DEGButton.Value==true
                app.Slider6.Value = deg2rad(app.J6.Value);
            end
            if app.RADButton.Value==true
                app.Slider6.Value = app.J6.Value;
            end
            app.config(6).JointPosition = app.configHome(6).JointPosition + app.Slider6.Value;
            showRobot(app);
            showCoordinates(app);
        end

        % Button pushed function: ButtonHOME
        function ButtonHOMEPushed(app, event)
            app.config = homeConfiguration(app.robot);
            app.Slider1.Value = 0;
            app.Slider2.Value = 0;
            app.Slider3.Value = 0;
            app.Slider4.Value = 0;
            app.Slider5.Value = 0;
            app.Slider6.Value = 0;
            showRobot(app);
            showCoordinates(app);            
        end

        % Selection changed function: ButtonGroup
        function ButtonGroupSelectionChanged(app, event)
            showCoordinates(app);  
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 2x1 grid
                app.GridLayout.RowHeight = {480, 480};
                app.GridLayout.ColumnWidth = {'1x'};
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 1;
            else
                % Change to a 1x2 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {234, '1x'};
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 2;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 835 480];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.Resize = 'off';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);
            app.UIFigure.Pointer = 'hand';

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {234, '1x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.BackgroundColor = [0.502 0.502 0.502];
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;
            app.LeftPanel.FontName = 'Arial';

            % Create J1Label
            app.J1Label = uilabel(app.LeftPanel);
            app.J1Label.BackgroundColor = [0.149 0.149 0.149];
            app.J1Label.HorizontalAlignment = 'center';
            app.J1Label.FontName = 'Arial';
            app.J1Label.FontWeight = 'bold';
            app.J1Label.FontColor = [1 1 1];
            app.J1Label.Position = [7 418 25 46];
            app.J1Label.Text = {'J1'; ''};

            % Create Slider1
            app.Slider1 = uislider(app.LeftPanel);
            app.Slider1.Limits = [-6.28318530717959 6.28318530717959];
            app.Slider1.MajorTicks = [-6.28318530717959 -4.71238898038469 -3.14159265358979 -1.5707963267949 0 1.5707963267949 3.14159265358979 4.71238898038469 6.28318530717959];
            app.Slider1.MajorTickLabels = {'-2&#960;', '-3&#960;/2', '-&#960;', '-&#960;/2', '0', '&#960;/2', '&#960;', '3&#960;/2', '2&#960;'};
            app.Slider1.ValueChangingFcn = createCallbackFcn(app, @Slider1ValueChanging, true);
            app.Slider1.Tooltip = {'Articulación J1'};
            app.Slider1.FontName = 'Arial';
            app.Slider1.FontSize = 8;
            app.Slider1.Position = [42 448 131 3];

            % Create J2Label
            app.J2Label = uilabel(app.LeftPanel);
            app.J2Label.BackgroundColor = [0.149 0.149 0.149];
            app.J2Label.HorizontalAlignment = 'center';
            app.J2Label.FontName = 'Arial';
            app.J2Label.FontWeight = 'bold';
            app.J2Label.FontColor = [1 1 1];
            app.J2Label.Position = [7 369 25 46];
            app.J2Label.Text = {'J2'; ''};

            % Create Slider2
            app.Slider2 = uislider(app.LeftPanel);
            app.Slider2.Limits = [-6.28318530717959 6.28318530717959];
            app.Slider2.MajorTicks = [-6.28318530717959 -4.71238898038469 -3.14159265358979 -1.5707963267949 0 1.5707963267949 3.14159265358979 4.71238898038469 6.28318530717959];
            app.Slider2.MajorTickLabels = {'-2&#960;', '-3&#960;/2', '-&#960;', '-&#960;/2', '0', '&#960;/2', '&#960;', '3&#960;/2', '2&#960;'};
            app.Slider2.ValueChangingFcn = createCallbackFcn(app, @Slider2ValueChanging, true);
            app.Slider2.Tooltip = {'Articulación J2'};
            app.Slider2.FontName = 'Arial';
            app.Slider2.FontSize = 8;
            app.Slider2.Position = [41 399 132 3];

            % Create J3Label
            app.J3Label = uilabel(app.LeftPanel);
            app.J3Label.BackgroundColor = [0.149 0.149 0.149];
            app.J3Label.HorizontalAlignment = 'center';
            app.J3Label.FontName = 'Arial';
            app.J3Label.FontWeight = 'bold';
            app.J3Label.FontColor = [1 1 1];
            app.J3Label.Position = [7 320 25 46];
            app.J3Label.Text = {'J3'; ''};

            % Create Slider3
            app.Slider3 = uislider(app.LeftPanel);
            app.Slider3.Limits = [-6.28318530717959 6.28318530717959];
            app.Slider3.MajorTicks = [-6.28318530717959 -4.71238898038469 -3.14159265358979 -1.5707963267949 0 1.5707963267949 3.14159265358979 4.71238898038469 6.28318530717959];
            app.Slider3.MajorTickLabels = {'-2&#960;', '-3&#960;/2', '-&#960;', '-&#960;/2', '0', '&#960;/2', '&#960;', '3&#960;/2', '2&#960;'};
            app.Slider3.ValueChangingFcn = createCallbackFcn(app, @Slider3ValueChanging, true);
            app.Slider3.Tooltip = {'Articulación J3'};
            app.Slider3.FontName = 'Arial';
            app.Slider3.FontSize = 8;
            app.Slider3.Position = [41 350 132 3];

            % Create J4Label
            app.J4Label = uilabel(app.LeftPanel);
            app.J4Label.BackgroundColor = [0.149 0.149 0.149];
            app.J4Label.HorizontalAlignment = 'center';
            app.J4Label.FontName = 'Arial';
            app.J4Label.FontWeight = 'bold';
            app.J4Label.FontColor = [1 1 1];
            app.J4Label.Position = [7 271 25 46];
            app.J4Label.Text = 'J4';

            % Create Slider4
            app.Slider4 = uislider(app.LeftPanel);
            app.Slider4.Limits = [-6.28318530717959 6.28318530717959];
            app.Slider4.MajorTicks = [-6.28318530717959 -4.71238898038469 -3.14159265358979 -1.5707963267949 0 1.5707963267949 3.14159265358979 4.71238898038469 6.28318530717959];
            app.Slider4.MajorTickLabels = {'-2&#960;', '-3&#960;/2', '-&#960;', '-&#960;/2', '0', '&#960;/2', '&#960;', '3&#960;/2', '2&#960;'};
            app.Slider4.ValueChangingFcn = createCallbackFcn(app, @Slider4ValueChanging, true);
            app.Slider4.Tooltip = {'Articulación J4'};
            app.Slider4.FontName = 'Arial';
            app.Slider4.FontSize = 8;
            app.Slider4.Position = [41 301 132 3];

            % Create J5Label
            app.J5Label = uilabel(app.LeftPanel);
            app.J5Label.BackgroundColor = [0.149 0.149 0.149];
            app.J5Label.HorizontalAlignment = 'center';
            app.J5Label.FontName = 'Arial';
            app.J5Label.FontWeight = 'bold';
            app.J5Label.FontColor = [1 1 1];
            app.J5Label.Position = [7 222 25 46];
            app.J5Label.Text = {'J5'; ''};

            % Create Slider5
            app.Slider5 = uislider(app.LeftPanel);
            app.Slider5.Limits = [-6.28318530717959 6.28318530717959];
            app.Slider5.MajorTicks = [-6.28318530717959 -4.71238898038469 -3.14159265358979 -1.5707963267949 0 1.5707963267949 3.14159265358979 4.71238898038469 6.28318530717959];
            app.Slider5.MajorTickLabels = {'-2&#960;', '-3&#960;/2', '-&#960;', '-&#960;/2', '0', '&#960;/2', '&#960;', '3&#960;/2', '2&#960;'};
            app.Slider5.ValueChangingFcn = createCallbackFcn(app, @Slider5ValueChanging, true);
            app.Slider5.Tooltip = {'Articulación J5'};
            app.Slider5.FontName = 'Arial';
            app.Slider5.FontSize = 8;
            app.Slider5.Position = [41 252 132 3];

            % Create J6Label
            app.J6Label = uilabel(app.LeftPanel);
            app.J6Label.BackgroundColor = [0.149 0.149 0.149];
            app.J6Label.HorizontalAlignment = 'center';
            app.J6Label.FontName = 'Arial';
            app.J6Label.FontWeight = 'bold';
            app.J6Label.FontColor = [1 1 1];
            app.J6Label.Position = [7 173 25 46];
            app.J6Label.Text = {'J6'; ''};

            % Create Slider6
            app.Slider6 = uislider(app.LeftPanel);
            app.Slider6.Limits = [-6.28318530717959 6.28318530717959];
            app.Slider6.MajorTicks = [-6.28318530717959 -4.71238898038469 -3.14159265358979 -1.5707963267949 0 1.5707963267949 3.14159265358979 4.71238898038469 6.28318530717959];
            app.Slider6.MajorTickLabels = {'-2&#960;', '-3&#960;/2', '-&#960;', '-&#960;/2', '0', '&#960;/2', '&#960;', '3&#960;/2', '2&#960;'};
            app.Slider6.ValueChangingFcn = createCallbackFcn(app, @Slider6ValueChanging, true);
            app.Slider6.Tooltip = {'Articulación J6'};
            app.Slider6.FontName = 'Arial';
            app.Slider6.FontSize = 8;
            app.Slider6.Position = [41 203 132 3];

            % Create XLabel
            app.XLabel = uilabel(app.LeftPanel);
            app.XLabel.BackgroundColor = [1 0 0];
            app.XLabel.HorizontalAlignment = 'center';
            app.XLabel.FontName = 'Arial';
            app.XLabel.FontSize = 14;
            app.XLabel.FontWeight = 'bold';
            app.XLabel.Position = [17 125 16 21];
            app.XLabel.Text = 'X';

            % Create X
            app.X = uilabel(app.LeftPanel);
            app.X.BackgroundColor = [0.902 0.902 0.902];
            app.X.HorizontalAlignment = 'center';
            app.X.FontName = 'Arial';
            app.X.FontColor = [1 0 0];
            app.X.Position = [33 124 45 22];
            app.X.Text = '0';

            % Create YLabel
            app.YLabel = uilabel(app.LeftPanel);
            app.YLabel.BackgroundColor = [0.1098 0.502 0.0902];
            app.YLabel.HorizontalAlignment = 'center';
            app.YLabel.FontName = 'Arial';
            app.YLabel.FontSize = 14;
            app.YLabel.FontWeight = 'bold';
            app.YLabel.Position = [88 124 17 22];
            app.YLabel.Text = 'Y';

            % Create Y
            app.Y = uilabel(app.LeftPanel);
            app.Y.BackgroundColor = [0.902 0.902 0.902];
            app.Y.HorizontalAlignment = 'center';
            app.Y.FontName = 'Arial';
            app.Y.FontColor = [0.1098 0.502 0.0902];
            app.Y.Position = [105 124 45 22];
            app.Y.Text = '0';

            % Create ZLabel
            app.ZLabel = uilabel(app.LeftPanel);
            app.ZLabel.BackgroundColor = [0 0.451 0.7412];
            app.ZLabel.HorizontalAlignment = 'center';
            app.ZLabel.FontName = 'Arial';
            app.ZLabel.FontSize = 14;
            app.ZLabel.FontWeight = 'bold';
            app.ZLabel.Position = [160 124 17 22];
            app.ZLabel.Text = 'Z';

            % Create Z
            app.Z = uilabel(app.LeftPanel);
            app.Z.BackgroundColor = [0.902 0.902 0.902];
            app.Z.HorizontalAlignment = 'center';
            app.Z.FontName = 'Arial';
            app.Z.FontColor = [0 0.451 0.7412];
            app.Z.Position = [177 124 45 22];
            app.Z.Text = '0';

            % Create ROLLLabel
            app.ROLLLabel = uilabel(app.LeftPanel);
            app.ROLLLabel.BackgroundColor = [0 0 0];
            app.ROLLLabel.HorizontalAlignment = 'center';
            app.ROLLLabel.FontName = 'Arial';
            app.ROLLLabel.FontSize = 14;
            app.ROLLLabel.FontWeight = 'bold';
            app.ROLLLabel.FontColor = [1 1 1];
            app.ROLLLabel.Position = [18 89 16 22];
            app.ROLLLabel.Text = 'R';

            % Create ROLL
            app.ROLL = uilabel(app.LeftPanel);
            app.ROLL.BackgroundColor = [0.902 0.902 0.902];
            app.ROLL.HorizontalAlignment = 'center';
            app.ROLL.FontName = 'Arial';
            app.ROLL.Position = [33 89 45 22];
            app.ROLL.Text = '0';

            % Create PITCHLabel
            app.PITCHLabel = uilabel(app.LeftPanel);
            app.PITCHLabel.BackgroundColor = [0 0 0];
            app.PITCHLabel.HorizontalAlignment = 'center';
            app.PITCHLabel.FontName = 'Arial';
            app.PITCHLabel.FontSize = 14;
            app.PITCHLabel.FontWeight = 'bold';
            app.PITCHLabel.FontColor = [1 1 1];
            app.PITCHLabel.Position = [88 89 18 22];
            app.PITCHLabel.Text = 'P';

            % Create PITCH
            app.PITCH = uilabel(app.LeftPanel);
            app.PITCH.BackgroundColor = [0.902 0.902 0.902];
            app.PITCH.HorizontalAlignment = 'center';
            app.PITCH.FontName = 'Arial';
            app.PITCH.Position = [105 89 45 22];
            app.PITCH.Text = '0';

            % Create YAWLabel
            app.YAWLabel = uilabel(app.LeftPanel);
            app.YAWLabel.BackgroundColor = [0 0 0];
            app.YAWLabel.HorizontalAlignment = 'center';
            app.YAWLabel.FontName = 'Arial';
            app.YAWLabel.FontSize = 14;
            app.YAWLabel.FontWeight = 'bold';
            app.YAWLabel.FontColor = [1 1 1];
            app.YAWLabel.Position = [160 89 17 22];
            app.YAWLabel.Text = 'Y';

            % Create YAW
            app.YAW = uilabel(app.LeftPanel);
            app.YAW.BackgroundColor = [0.902 0.902 0.902];
            app.YAW.HorizontalAlignment = 'center';
            app.YAW.FontName = 'Arial';
            app.YAW.Position = [176 89 45 22];
            app.YAW.Text = '0';

            % Create J1
            app.J1 = uieditfield(app.LeftPanel, 'numeric');
            app.J1.Limits = [-360 360];
            app.J1.ValueDisplayFormat = '%5.1f';
            app.J1.ValueChangedFcn = createCallbackFcn(app, @J1ValueChanged, true);
            app.J1.HorizontalAlignment = 'center';
            app.J1.FontName = 'Arial';
            app.J1.Position = [184 438 45 22];

            % Create BASELabel
            app.BASELabel = uilabel(app.LeftPanel);
            app.BASELabel.BackgroundColor = [0.8 0.8 0.8];
            app.BASELabel.HorizontalAlignment = 'center';
            app.BASELabel.FontName = 'Arial';
            app.BASELabel.FontSize = 7;
            app.BASELabel.FontWeight = 'bold';
            app.BASELabel.Position = [184 426 45 12];
            app.BASELabel.Text = 'BASE';

            % Create SHOULDERLabel
            app.SHOULDERLabel = uilabel(app.LeftPanel);
            app.SHOULDERLabel.BackgroundColor = [0.8 0.8 0.8];
            app.SHOULDERLabel.HorizontalAlignment = 'center';
            app.SHOULDERLabel.FontName = 'Arial';
            app.SHOULDERLabel.FontSize = 7;
            app.SHOULDERLabel.FontWeight = 'bold';
            app.SHOULDERLabel.Position = [184 377 45 12];
            app.SHOULDERLabel.Text = {'SHOULDER'; ''};

            % Create J2
            app.J2 = uieditfield(app.LeftPanel, 'numeric');
            app.J2.Limits = [-360 360];
            app.J2.ValueDisplayFormat = '%5.1f';
            app.J2.ValueChangedFcn = createCallbackFcn(app, @J2ValueChanged, true);
            app.J2.HorizontalAlignment = 'center';
            app.J2.FontName = 'Arial';
            app.J2.Position = [184 389 45 22];

            % Create ELBOWLabel
            app.ELBOWLabel = uilabel(app.LeftPanel);
            app.ELBOWLabel.BackgroundColor = [0.8 0.8 0.8];
            app.ELBOWLabel.HorizontalAlignment = 'center';
            app.ELBOWLabel.FontName = 'Arial';
            app.ELBOWLabel.FontSize = 7;
            app.ELBOWLabel.FontWeight = 'bold';
            app.ELBOWLabel.Position = [184 328 45 12];
            app.ELBOWLabel.Text = 'ELBOW';

            % Create J3
            app.J3 = uieditfield(app.LeftPanel, 'numeric');
            app.J3.Limits = [-360 360];
            app.J3.ValueDisplayFormat = '%5.1f';
            app.J3.ValueChangedFcn = createCallbackFcn(app, @J3ValueChanged, true);
            app.J3.HorizontalAlignment = 'center';
            app.J3.FontName = 'Arial';
            app.J3.Position = [184 340 45 22];

            % Create WRIST1Label
            app.WRIST1Label = uilabel(app.LeftPanel);
            app.WRIST1Label.BackgroundColor = [0.8 0.8 0.8];
            app.WRIST1Label.HorizontalAlignment = 'center';
            app.WRIST1Label.FontName = 'Arial';
            app.WRIST1Label.FontSize = 7;
            app.WRIST1Label.FontWeight = 'bold';
            app.WRIST1Label.Position = [184 279 45 12];
            app.WRIST1Label.Text = 'WRIST 1';

            % Create J4
            app.J4 = uieditfield(app.LeftPanel, 'numeric');
            app.J4.Limits = [-360 360];
            app.J4.ValueDisplayFormat = '%5.1f';
            app.J4.ValueChangedFcn = createCallbackFcn(app, @J4ValueChanged, true);
            app.J4.HorizontalAlignment = 'center';
            app.J4.FontName = 'Arial';
            app.J4.Position = [184 291 45 22];

            % Create WRIST2Label
            app.WRIST2Label = uilabel(app.LeftPanel);
            app.WRIST2Label.BackgroundColor = [0.8 0.8 0.8];
            app.WRIST2Label.HorizontalAlignment = 'center';
            app.WRIST2Label.FontName = 'Arial';
            app.WRIST2Label.FontSize = 7;
            app.WRIST2Label.FontWeight = 'bold';
            app.WRIST2Label.Position = [184 230 45 12];
            app.WRIST2Label.Text = {'WRIST 2'; ''};

            % Create J5
            app.J5 = uieditfield(app.LeftPanel, 'numeric');
            app.J5.Limits = [-360 360];
            app.J5.ValueDisplayFormat = '%5.1f';
            app.J5.ValueChangedFcn = createCallbackFcn(app, @J5ValueChanged, true);
            app.J5.HorizontalAlignment = 'center';
            app.J5.FontName = 'Arial';
            app.J5.Position = [184 242 45 22];

            % Create WRIST3Label
            app.WRIST3Label = uilabel(app.LeftPanel);
            app.WRIST3Label.BackgroundColor = [0.8 0.8 0.8];
            app.WRIST3Label.HorizontalAlignment = 'center';
            app.WRIST3Label.FontName = 'Arial';
            app.WRIST3Label.FontSize = 7;
            app.WRIST3Label.FontWeight = 'bold';
            app.WRIST3Label.Position = [184 181 45 12];
            app.WRIST3Label.Text = 'WRIST 3';

            % Create J6
            app.J6 = uieditfield(app.LeftPanel, 'numeric');
            app.J6.Limits = [-360 360];
            app.J6.ValueDisplayFormat = '%5.1f';
            app.J6.ValueChangedFcn = createCallbackFcn(app, @J6ValueChanged, true);
            app.J6.HorizontalAlignment = 'center';
            app.J6.FontName = 'Arial';
            app.J6.Position = [184 193 45 22];

            % Create ButtonHOME
            app.ButtonHOME = uibutton(app.LeftPanel, 'push');
            app.ButtonHOME.ButtonPushedFcn = createCallbackFcn(app, @ButtonHOMEPushed, true);
            app.ButtonHOME.Icon = 'home.png';
            app.ButtonHOME.BackgroundColor = [0.902 0.902 0.902];
            app.ButtonHOME.FontName = 'Arial';
            app.ButtonHOME.FontWeight = 'bold';
            app.ButtonHOME.Position = [17 18 41 39];
            app.ButtonHOME.Text = '';

            % Create TEAM
            app.TEAM = uilabel(app.LeftPanel);
            app.TEAM.BackgroundColor = [0.8 0.8 0.8];
            app.TEAM.FontName = 'Arial';
            app.TEAM.FontSize = 9;
            app.TEAM.Position = [68 15 153 30];
            app.TEAM.Text = {' Jose Jairo Guitterez Moreno'; ' Henry Omar Moreno Jiménez'; ' John Sebastian Panche Estupiñán'};

            % Create AUTOR
            app.AUTOR = uilabel(app.LeftPanel);
            app.AUTOR.BackgroundColor = [0.651 0.651 0.651];
            app.AUTOR.FontName = 'Arial';
            app.AUTOR.FontSize = 8;
            app.AUTOR.FontWeight = 'bold';
            app.AUTOR.FontColor = [0.0431 0.2118 0.3216];
            app.AUTOR.Position = [68 44 153 14];
            app.AUTOR.Text = ' Desarrollado por:';

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;

            % Create UIAxes
            app.UIAxes = uiaxes(app.RightPanel);
            title(app.UIAxes, 'UNIVERSAL ROBOTS UR5')
            xlabel(app.UIAxes, '')
            ylabel(app.UIAxes, '')
            app.UIAxes.FontName = 'Arial';
            app.UIAxes.FontSize = 10;
            app.UIAxes.XColor = [0.9412 0.9412 0.9412];
            app.UIAxes.XTick = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
            app.UIAxes.XTickLabel = '';
            app.UIAxes.YColor = [0.9412 0.9412 0.9412];
            app.UIAxes.YTick = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
            app.UIAxes.YTickLabel = '';
            app.UIAxes.ZColor = [0.9412 0.9412 0.9412];
            app.UIAxes.Color = [0.9412 0.9412 0.9412];
            app.UIAxes.Position = [7 6 588 468];

            % Create ButtonGroup
            app.ButtonGroup = uibuttongroup(app.RightPanel);
            app.ButtonGroup.SelectionChangedFcn = createCallbackFcn(app, @ButtonGroupSelectionChanged, true);
            app.ButtonGroup.BorderType = 'none';
            app.ButtonGroup.BackgroundColor = [0.902 0.902 0.902];
            app.ButtonGroup.FontName = 'Arial';
            app.ButtonGroup.Position = [8 454 105 18];

            % Create DEGButton
            app.DEGButton = uiradiobutton(app.ButtonGroup);
            app.DEGButton.Text = 'DEG';
            app.DEGButton.FontName = 'Arial';
            app.DEGButton.FontSize = 8;
            app.DEGButton.FontWeight = 'bold';
            app.DEGButton.Position = [9 -1 46 22];
            app.DEGButton.Value = true;

            % Create RADButton
            app.RADButton = uiradiobutton(app.ButtonGroup);
            app.RADButton.Text = 'RAD';
            app.RADButton.FontName = 'Arial';
            app.RADButton.FontSize = 8;
            app.RADButton.FontWeight = 'bold';
            app.RADButton.Position = [56 -1 48 22];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = GUI_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end