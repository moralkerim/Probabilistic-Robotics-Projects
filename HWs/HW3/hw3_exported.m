classdef hw3_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure               matlab.ui.Figure
        SensorPanel            matlab.ui.container.Panel
        NavigationPanel        matlab.ui.container.Panel
        MainControlPanel       matlab.ui.container.Panel
        ExecuteButton          matlab.ui.control.Button
        MotionPanel            matlab.ui.container.Panel
        alpha1SliderLabel      matlab.ui.control.Label
        alpha1Slider           matlab.ui.control.Slider
        alpha2SliderLabel      matlab.ui.control.Label
        alpha2Slider           matlab.ui.control.Slider
        alpha3SliderLabel      matlab.ui.control.Label
        alpha3Slider           matlab.ui.control.Slider
        alpha4SliderLabel      matlab.ui.control.Label
        alpha4Slider           matlab.ui.control.Slider
        MethodButtonGroup      matlab.ui.container.ButtonGroup
        DirectEvButton         matlab.ui.control.RadioButton
        SamplingButton         matlab.ui.control.RadioButton
        ModelButtonGroup       matlab.ui.container.ButtonGroup
        DeadReckButton         matlab.ui.control.RadioButton
        OdometryButton         matlab.ui.control.RadioButton
        alpha5SliderLabel      matlab.ui.control.Label
        alpha5Slider           matlab.ui.control.Slider
        alpha6SliderLabel      matlab.ui.control.Label
        alpha6Slider           matlab.ui.control.Slider
        TrajectoryButtonGroup  matlab.ui.container.ButtonGroup
        CircularButton         matlab.ui.control.RadioButton
        RectengularButton      matlab.ui.control.RadioButton
        UIAxes                 matlab.ui.control.UIAxes
    end

    
    methods (Access = private)
        
            
    function sample = normal_dist(app,a1, a2, v,w)
        b = (a1 * abs(v) + a2 * abs(w));
        ran_num  = -b + (b+b).*rand(12,1);
        sum = 0;
        for i=1:12
            sum = sum + ran_num(i);
        end
        sample = 0.5 * sum;
    end
    
    function Odometry(app)
    a1 = app.alpha1Slider.Value;  a2 = app.alpha2Slider.Value;
    a3 = app.alpha3Slider.Value;  a4 = app.alpha4Slider.Value;
    a5 = app.alpha5Slider.Value;  a6 = app.alpha6Slider.Value; 
    if(app.CircularButton.Value == 1)
        sample_n = 150;
        xc = 0; yc = 0;
        x=zeros(sample_n,8); y=zeros(sample_n,8); tet = zeros(sample_n,8);
        xp=zeros(sample_n,8); yp=zeros(sample_n,8); tetp = zeros(sample_n,8);
        xp(:,1) = 1; yp(:,1) = 0; tetp(:,1) = -pi/2;
        tet(:,1) = -pi/2;
        r = 1;
        t = 1; %time of traviling for 45 degrees
        w = -pi/4;
        for i=1:8
              x(:,i) = xc - r*sin(tet(1,i));
              y(:,i) = yc + r*cos(tet(1,i));
              tet(1,i+1) = tet(1,i) + w*t;
        end
            for i=1:7
                for j=1:sample_n
                    delta_rot1 = atan2((y(j,i+1)-y(j,i)),(x(j,i+1)-x(j,i))) - tet(j,i);
                    delta_trans = sqrt((y(j,i+1)-y(j,i))^2 + (x(j,i+1)-x(j,i))^2);
                    delta_rot2 = tet(j,i+1) - tet(j,i) - delta_rot1;
                    
                    delta_rot1_hat = delta_rot1 -  normal_dist(app,a1,a2,delta_rot1,delta_trans);
                    delta_trans_hat = delta_trans -  normal_dist(app,a3,a4,delta_trans,delta_rot1+delta_rot2);
                    delta_rot2_hat = delta_rot2 - normal_dist(app,a1,a2,delta_rot2,delta_trans);
                    
                    xp(j,i+1) = xp(j,i) + delta_trans_hat * cos(tetp(j,i)+delta_rot1_hat);
                    yp(j,i+1) = yp(j,i) + delta_trans_hat * sin(tetp(j,i)+delta_rot1_hat);
                    tetp(j,i+1) = tetp(j,i) + delta_rot1_hat + delta_rot2_hat;
                end
            end
        plot(app.UIAxes,xp,yp,'k.');        
    end
    if(app.RectengularButton.Value == 1)
        sample_n = 150;
        xc = 0; yc = 0;
        x=zeros(sample_n,13); y=zeros(sample_n,13); tet = zeros(sample_n,13);
        xp=zeros(sample_n,13); yp=zeros(sample_n,13); tetp = zeros(sample_n,13);
        x(:,1) = -4; y(:,1) = -4; tet(:,1) = 0;
        xp(:,1) = -4; yp(:,1) = -4; tetp(:,1) = 0;
        tet(:,1) = 0;
        r = 1;
        t = 1; %time of traviling for 45 degrees
        d2r = pi/180;
        w = 0.001; %Assumed 45 degrees travel in one sec.
        v = 2;
        for i=2:5
              x(:,i) = x(:,i-1) + 2;
              y(:,i) = y(1,1);
              tet(:,i) = 0;
        end
        
        for i=6:9
              x(:,i) = x(1,5);
              y(:,i) = y(:,i-1) + 2;
              tet(:,i) = pi/2;
        end
        
        for i=10:13
              x(:,i) = x(:,i-1) - 2;
              y(:,i) = 4;
              tet(:,i) = pi;
        end
        
            for i=1:12
                for j=1:sample_n
                    delta_rot1 = atan2((y(j,i+1)-y(j,i)),(x(j,i+1)-x(j,i))) - tet(j,i);
                    delta_trans = sqrt((y(j,i+1)-y(j,i))^2 + (x(j,i+1)-x(j,i))^2);
                    delta_rot2 = tet(j,i+1) - tet(j,i) - delta_rot1;
                    
                    delta_rot1_hat = delta_rot1 -  normal_dist(app,a1,a2,delta_rot1,delta_trans);
                    delta_trans_hat = delta_trans -  normal_dist(app,a3,a4,delta_trans,delta_rot1+delta_rot2);
                    delta_rot2_hat = delta_rot2 -  normal_dist(app,a1,a2,delta_rot2,delta_trans);
                    
                    xp(j,i+1) = xp(j,i) + delta_trans_hat * cos(tetp(j,i)+delta_rot1_hat);
                    yp(j,i+1) = yp(j,i) + delta_trans_hat * sin(tetp(j,i)+delta_rot1_hat);
                    tetp(j,i+1) = tetp(j,i) + delta_rot1_hat + delta_rot2_hat;
                end
            end
        plot(app.UIAxes,xp,yp,'k.');       
    end
    end
            
    function dead_reckoning(app)
    a1 = app.alpha1Slider.Value;  a2 = app.alpha2Slider.Value;
    a3 = app.alpha3Slider.Value;  a4 = app.alpha4Slider.Value;
    a5 = app.alpha5Slider.Value;  a6 = app.alpha6Slider.Value;
    if(app.CircularButton.Value == 1)
        sample_n = 150;
        x=zeros(sample_n,8); y=zeros(sample_n,8); tet = zeros(sample_n,8);
        x(:,1) = 1; tet(:,1) = pi/2;
        r = 1;
        t = 1; %time of traviling for 45 degrees
        w = -pi/4; %Assumed 45 degrees travel in one sec.
        v = w * r;
        plot(app.UIAxes,x(1,1),y(1,1),'k.','LineWidth',2);
        hold(app.UIAxes,"on");
          for i=1:7
                for j=1:sample_n
                    v_hat = v + normal_dist(app,a1, a2, v,w);
                    w_hat = w + normal_dist(app,a3, a4, v,w);
                    gama_hat = normal_dist(app,a5, a6, v,w);
                    x(j,i+1) = x(j,i) -  v_hat/w_hat * sin(tet(j,i)) +  v_hat/w_hat * sin(tet(j,i) + w_hat*t);
                    y(j,i+1) = y(j,i) +  v_hat/w_hat * cos(tet(j,i)) -  v_hat/w_hat * cos(tet(j,i) + w_hat*t);
                    tet(j,i+1) = tet(j,i) + w_hat*t + gama_hat*t;
                    plot(app.UIAxes,x(j,i+1),y(j,i+1),'k.','LineWidth',2);
                    hold(app.UIAxes,"on");
                end
           end
            hold(app.UIAxes,'off');
    end
    if(app.RectengularButton.Value == 1)
    sample_n = 100;
    x=zeros(sample_n,13); y=zeros(sample_n,13); tet = zeros(sample_n,13);
    x(:,1) = -4; y(:,1) = -4; tet(:,1) = 0; %Initial position.
    t = 1; %time of traviling for 2 meters
    v = 2; %Assumed traveled 2 meters in 1 sec.
    w = 0.001; %Assumed no rotation.
    plot(app.UIAxes,x(1,1),y(1,1),'k.','LineWidth',2);
    hold(app.UIAxes,"on");
        for i=1:12
            if(i == 4 || i == 8)
                w = pi/2;
            else
                w = 0.001;
            end
            for j=1:sample_n
                v_hat = v + normal_dist(app,a1, a2, v,w);
                w_hat = w + normal_dist(app,a3, a4, v,w);
                gama_hat = normal_dist(app,a5, a6, v,w);
                x(j,i+1) = x(j,i) -  v_hat/w_hat * sin(tet(j,i)) +  v_hat/w_hat * sin(tet(j,i) + w_hat*t);
                y(j,i+1) = y(j,i) +  v_hat/w_hat * cos(tet(j,i)) -  v_hat/w_hat * cos(tet(j,i) + w_hat*t);
                tet(j,i+1) = tet(j,i) + w_hat*t + gama_hat*t;
                plot(app.UIAxes,x(j,i+1),y(j,i+1),'k.','LineWidth',2);
                hold(app.UIAxes,'on');
            end
        end
        hold(app.UIAxes,'off');
    end
    end
   end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: ExecuteButton
        function ExecuteButtonPushed(app, event)
            if(app.DeadReckButton.Value == 1)
                dead_reckoning(app);
            end
            if(app.OdometryButton.Value == 1)
                Odometry(app);
                xlabel(app.UIAxes,'Test');
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 814 700];
            app.UIFigure.Name = 'MATLAB App';

            % Create SensorPanel
            app.SensorPanel = uipanel(app.UIFigure);
            app.SensorPanel.Title = 'Sensor';
            app.SensorPanel.Position = [574 423 215 250];

            % Create NavigationPanel
            app.NavigationPanel = uipanel(app.UIFigure);
            app.NavigationPanel.Title = 'Navigation';
            app.NavigationPanel.Position = [574 180 215 231];

            % Create MainControlPanel
            app.MainControlPanel = uipanel(app.UIFigure);
            app.MainControlPanel.Title = 'Main Control';
            app.MainControlPanel.Position = [574 16 215 131];

            % Create ExecuteButton
            app.ExecuteButton = uibutton(app.MainControlPanel, 'push');
            app.ExecuteButton.ButtonPushedFcn = createCallbackFcn(app, @ExecuteButtonPushed, true);
            app.ExecuteButton.Position = [58 39 100 22];
            app.ExecuteButton.Text = 'Execute';

            % Create MotionPanel
            app.MotionPanel = uipanel(app.UIFigure);
            app.MotionPanel.Title = 'Motion';
            app.MotionPanel.Position = [22 1 524 306];

            % Create alpha1SliderLabel
            app.alpha1SliderLabel = uilabel(app.MotionPanel);
            app.alpha1SliderLabel.HorizontalAlignment = 'right';
            app.alpha1SliderLabel.FontSize = 10;
            app.alpha1SliderLabel.Position = [15 214 35 22];
            app.alpha1SliderLabel.Text = 'alpha1';

            % Create alpha1Slider
            app.alpha1Slider = uislider(app.MotionPanel);
            app.alpha1Slider.Limits = [0 0.1];
            app.alpha1Slider.FontSize = 10;
            app.alpha1Slider.Position = [71 223 110 3];
            app.alpha1Slider.Value = 0.01;

            % Create alpha2SliderLabel
            app.alpha2SliderLabel = uilabel(app.MotionPanel);
            app.alpha2SliderLabel.HorizontalAlignment = 'right';
            app.alpha2SliderLabel.FontSize = 10;
            app.alpha2SliderLabel.Position = [200 212 35 22];
            app.alpha2SliderLabel.Text = 'alpha2';

            % Create alpha2Slider
            app.alpha2Slider = uislider(app.MotionPanel);
            app.alpha2Slider.Limits = [0 0.1];
            app.alpha2Slider.FontSize = 10;
            app.alpha2Slider.Position = [256 221 110 3];
            app.alpha2Slider.Value = 0.01;

            % Create alpha3SliderLabel
            app.alpha3SliderLabel = uilabel(app.MotionPanel);
            app.alpha3SliderLabel.HorizontalAlignment = 'right';
            app.alpha3SliderLabel.FontSize = 10;
            app.alpha3SliderLabel.Position = [15 154 35 22];
            app.alpha3SliderLabel.Text = 'alpha3';

            % Create alpha3Slider
            app.alpha3Slider = uislider(app.MotionPanel);
            app.alpha3Slider.Limits = [0 0.1];
            app.alpha3Slider.FontSize = 10;
            app.alpha3Slider.Position = [71 163 110 3];
            app.alpha3Slider.Value = 0.01;

            % Create alpha4SliderLabel
            app.alpha4SliderLabel = uilabel(app.MotionPanel);
            app.alpha4SliderLabel.HorizontalAlignment = 'right';
            app.alpha4SliderLabel.FontSize = 10;
            app.alpha4SliderLabel.Position = [200 154 35 22];
            app.alpha4SliderLabel.Text = 'alpha4';

            % Create alpha4Slider
            app.alpha4Slider = uislider(app.MotionPanel);
            app.alpha4Slider.Limits = [0 0.1];
            app.alpha4Slider.FontSize = 10;
            app.alpha4Slider.Position = [256 163 110 3];
            app.alpha4Slider.Value = 0.01;

            % Create MethodButtonGroup
            app.MethodButtonGroup = uibuttongroup(app.MotionPanel);
            app.MethodButtonGroup.Title = 'Method';
            app.MethodButtonGroup.Position = [396 100 123 80];

            % Create DirectEvButton
            app.DirectEvButton = uiradiobutton(app.MethodButtonGroup);
            app.DirectEvButton.Text = 'Direct Ev.';
            app.DirectEvButton.Position = [11 34 73 22];
            app.DirectEvButton.Value = true;

            % Create SamplingButton
            app.SamplingButton = uiradiobutton(app.MethodButtonGroup);
            app.SamplingButton.Text = 'Sampling';
            app.SamplingButton.Position = [11 12 72 22];

            % Create ModelButtonGroup
            app.ModelButtonGroup = uibuttongroup(app.MotionPanel);
            app.ModelButtonGroup.Title = 'Model';
            app.ModelButtonGroup.Position = [396 10 123 80];

            % Create DeadReckButton
            app.DeadReckButton = uiradiobutton(app.ModelButtonGroup);
            app.DeadReckButton.Text = 'Dead-Reck.';
            app.DeadReckButton.Position = [11 34 85 22];
            app.DeadReckButton.Value = true;

            % Create OdometryButton
            app.OdometryButton = uiradiobutton(app.ModelButtonGroup);
            app.OdometryButton.Text = 'Odometry';
            app.OdometryButton.Position = [11 12 75 22];

            % Create alpha5SliderLabel
            app.alpha5SliderLabel = uilabel(app.MotionPanel);
            app.alpha5SliderLabel.HorizontalAlignment = 'right';
            app.alpha5SliderLabel.FontSize = 10;
            app.alpha5SliderLabel.Position = [15 86 35 22];
            app.alpha5SliderLabel.Text = 'alpha5';

            % Create alpha5Slider
            app.alpha5Slider = uislider(app.MotionPanel);
            app.alpha5Slider.Limits = [0 0.1];
            app.alpha5Slider.FontSize = 10;
            app.alpha5Slider.Position = [71 95 110 3];
            app.alpha5Slider.Value = 0.01;

            % Create alpha6SliderLabel
            app.alpha6SliderLabel = uilabel(app.MotionPanel);
            app.alpha6SliderLabel.HorizontalAlignment = 'right';
            app.alpha6SliderLabel.FontSize = 10;
            app.alpha6SliderLabel.Position = [200 86 35 22];
            app.alpha6SliderLabel.Text = 'alpha6';

            % Create alpha6Slider
            app.alpha6Slider = uislider(app.MotionPanel);
            app.alpha6Slider.Limits = [0 0.1];
            app.alpha6Slider.FontSize = 10;
            app.alpha6Slider.Position = [256 95 110 3];
            app.alpha6Slider.Value = 0.01;

            % Create TrajectoryButtonGroup
            app.TrajectoryButtonGroup = uibuttongroup(app.MotionPanel);
            app.TrajectoryButtonGroup.Title = 'Trajectory';
            app.TrajectoryButtonGroup.Position = [396 190 123 80];

            % Create CircularButton
            app.CircularButton = uiradiobutton(app.TrajectoryButtonGroup);
            app.CircularButton.Text = 'Circular';
            app.CircularButton.Position = [11 34 63 22];
            app.CircularButton.Value = true;

            % Create RectengularButton
            app.RectengularButton = uiradiobutton(app.TrajectoryButtonGroup);
            app.RectengularButton.Text = 'Rectengular';
            app.RectengularButton.Position = [11 12 87 22];

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Robot Space')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Position = [22 306 533 380];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = hw3_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

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