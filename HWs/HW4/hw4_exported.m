classdef hw4_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                matlab.ui.Figure
        SensorPanel             matlab.ui.container.Panel
        dmEditFieldLabel        matlab.ui.control.Label
        dmEditField             matlab.ui.control.NumericEditField
        ThetadgsEditFieldLabel  matlab.ui.control.Label
        ThetadgsEditField       matlab.ui.control.NumericEditField
        ResolutmEditFieldLabel  matlab.ui.control.Label
        ResolutmEditField       matlab.ui.control.NumericEditField
        sigma_dSliderLabel      matlab.ui.control.Label
        sigma_dSlider           matlab.ui.control.Slider
        sigma_thetSliderLabel   matlab.ui.control.Label
        sigma_thetSlider        matlab.ui.control.Slider
        sigma_sSliderLabel      matlab.ui.control.Label
        sigma_sSlider           matlab.ui.control.Slider
        NavigationPanel         matlab.ui.container.Panel
        MainControlPanel        matlab.ui.container.Panel
        ExecuteButton           matlab.ui.control.Button
        MotionPanel             matlab.ui.container.Panel
        alpha1SliderLabel       matlab.ui.control.Label
        alpha1Slider            matlab.ui.control.Slider
        alpha2SliderLabel       matlab.ui.control.Label
        alpha2Slider            matlab.ui.control.Slider
        alpha3SliderLabel       matlab.ui.control.Label
        alpha3Slider            matlab.ui.control.Slider
        alpha4SliderLabel       matlab.ui.control.Label
        alpha4Slider            matlab.ui.control.Slider
        MethodButtonGroup       matlab.ui.container.ButtonGroup
        DirectEvButton          matlab.ui.control.RadioButton
        SamplingButton          matlab.ui.control.RadioButton
        ModelButtonGroup        matlab.ui.container.ButtonGroup
        DeadReckButton          matlab.ui.control.RadioButton
        OdometryButton          matlab.ui.control.RadioButton
        alpha5SliderLabel       matlab.ui.control.Label
        alpha5Slider            matlab.ui.control.Slider
        alpha6SliderLabel       matlab.ui.control.Label
        alpha6Slider            matlab.ui.control.Slider
        TrajectoryButtonGroup   matlab.ui.container.ButtonGroup
        CircularButton          matlab.ui.control.RadioButton
        RectengularButton       matlab.ui.control.RadioButton
        LandmarkPanel           matlab.ui.container.Panel
        SigNumEditFieldLabel    matlab.ui.control.Label
        SigNumEditField         matlab.ui.control.NumericEditField
        SignatureDropDownLabel  matlab.ui.control.Label
        SignatureDropDown       matlab.ui.control.DropDown
        MxmEditFieldLabel       matlab.ui.control.Label
        MxmEditField            matlab.ui.control.NumericEditField
        MymEditFieldLabel       matlab.ui.control.Label
        MymEditField            matlab.ui.control.NumericEditField
        UIAxes                  matlab.ui.control.UIAxes
    end

    
    properties (Access = public)
        sign1 % Description
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
    
    function sample1 = normal_dist1(app,b)
        ran_num  = -b + (b+b).*rand(12,1);
        sum = 0;
        for i=1:12
            sum = sum + ran_num(i);
        end
        sample1 = 0.5 * sum;
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
                   
                    d = sqrt((xp(j,i+1) - app.sign1.Mx)^2 + (yp(j,i+1) - app.sign1.My)^2) + normal_dist1(app,app.sigma_dSlider.Value);
                    fi = atan2(app.sign1.Mx-xp(j,i+1), app.sign1.My-yp(j,i+1))-tetp(j,i+1) + normal_dist1(app,app.sigma_thetSlider.Value);
                    
                    if(d <= app.dmEditField.Value && abs(fi) <= app.ThetadgsEditField.Value )
                        plot(app.UIAxes, xp(j,i+1)+d*cos(tetp(j,i+1)), yp(j,i+1)+d*sin(tetp(j,i+1)), 'r.');
                        hold(app.UIAxes, "on");
                    end
                    
                end
            end
        plot(app.UIAxes,xp,yp,'k.');  
        plot(app.UIAxes,app.sign1.Mx,app.sign1.My,'bo', 'LineWidth', 10);
        hold(app.UIAxes, "off");
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

        % Code that executes after component creation
        function startupFcn(app)
            app.sign1 = Signature;
        end

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

        % Value changed function: SigNumEditField
        function SigNumEditFieldValueChanged(app, event)

        sig_num = app.SigNumEditField.Value;
        item_array = string(zeros(1,sig_num));
        sign_array = cell(1,sig_num);
        for i=1:sig_num
            item_array(i) = string(i);
        end
        app.SignatureDropDown.Items = item_array;
        
        end

        % Value changed function: MxmEditField
        function MxmEditFieldValueChanged(app, event)
            mx = app.MxmEditField.Value;
            app.sign1.Mx = mx;
        end

        % Value changed function: MymEditField
        function MymEditFieldValueChanged(app, event)
            my = app.MymEditField.Value;
            app.sign1.My = my;
            plot(app.UIAxes,app.sign1.Mx,app.sign1.My,'bo', 'LineWidth', 10);
            hold(app.UIAxes,'on');
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

            % Create dmEditFieldLabel
            app.dmEditFieldLabel = uilabel(app.SensorPanel);
            app.dmEditFieldLabel.HorizontalAlignment = 'right';
            app.dmEditFieldLabel.Position = [42 197 37 22];
            app.dmEditFieldLabel.Text = 'd (m) ';

            % Create dmEditField
            app.dmEditField = uieditfield(app.SensorPanel, 'numeric');
            app.dmEditField.Position = [94 197 100 22];

            % Create ThetadgsEditFieldLabel
            app.ThetadgsEditFieldLabel = uilabel(app.SensorPanel);
            app.ThetadgsEditFieldLabel.HorizontalAlignment = 'right';
            app.ThetadgsEditFieldLabel.Position = [15 167 67 22];
            app.ThetadgsEditFieldLabel.Text = 'Theta (dgs)';

            % Create ThetadgsEditField
            app.ThetadgsEditField = uieditfield(app.SensorPanel, 'numeric');
            app.ThetadgsEditField.Position = [97 167 100 22];

            % Create ResolutmEditFieldLabel
            app.ResolutmEditFieldLabel = uilabel(app.SensorPanel);
            app.ResolutmEditFieldLabel.HorizontalAlignment = 'right';
            app.ResolutmEditFieldLabel.Position = [8 134 71 22];
            app.ResolutmEditFieldLabel.Text = 'Resolut. (m)';

            % Create ResolutmEditField
            app.ResolutmEditField = uieditfield(app.SensorPanel, 'numeric');
            app.ResolutmEditField.Position = [94 134 100 22];

            % Create sigma_dSliderLabel
            app.sigma_dSliderLabel = uilabel(app.SensorPanel);
            app.sigma_dSliderLabel.HorizontalAlignment = 'right';
            app.sigma_dSliderLabel.FontSize = 10;
            app.sigma_dSliderLabel.Position = [11 105 43 22];
            app.sigma_dSliderLabel.Text = 'sigma_d';

            % Create sigma_dSlider
            app.sigma_dSlider = uislider(app.SensorPanel);
            app.sigma_dSlider.Limits = [0 1];
            app.sigma_dSlider.FontSize = 10;
            app.sigma_dSlider.Position = [75 114 110 3];

            % Create sigma_thetSliderLabel
            app.sigma_thetSliderLabel = uilabel(app.SensorPanel);
            app.sigma_thetSliderLabel.HorizontalAlignment = 'right';
            app.sigma_thetSliderLabel.FontSize = 10;
            app.sigma_thetSliderLabel.Position = [1 66 55 22];
            app.sigma_thetSliderLabel.Text = 'sigma_thet';

            % Create sigma_thetSlider
            app.sigma_thetSlider = uislider(app.SensorPanel);
            app.sigma_thetSlider.Limits = [0 1];
            app.sigma_thetSlider.FontSize = 10;
            app.sigma_thetSlider.Position = [77 75 110 3];

            % Create sigma_sSliderLabel
            app.sigma_sSliderLabel = uilabel(app.SensorPanel);
            app.sigma_sSliderLabel.HorizontalAlignment = 'right';
            app.sigma_sSliderLabel.FontSize = 10;
            app.sigma_sSliderLabel.Position = [13 25 43 22];
            app.sigma_sSliderLabel.Text = 'sigma_s';

            % Create sigma_sSlider
            app.sigma_sSlider = uislider(app.SensorPanel);
            app.sigma_sSlider.Limits = [0 1];
            app.sigma_sSlider.FontSize = 10;
            app.sigma_sSlider.Position = [77 34 110 3];

            % Create NavigationPanel
            app.NavigationPanel = uipanel(app.UIFigure);
            app.NavigationPanel.Title = 'Navigation';
            app.NavigationPanel.Position = [575 156 215 127];

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

            % Create SamplingButton
            app.SamplingButton = uiradiobutton(app.MethodButtonGroup);
            app.SamplingButton.Text = 'Sampling';
            app.SamplingButton.Position = [11 12 72 22];
            app.SamplingButton.Value = true;

            % Create ModelButtonGroup
            app.ModelButtonGroup = uibuttongroup(app.MotionPanel);
            app.ModelButtonGroup.Title = 'Model';
            app.ModelButtonGroup.Position = [396 10 123 80];

            % Create DeadReckButton
            app.DeadReckButton = uiradiobutton(app.ModelButtonGroup);
            app.DeadReckButton.Text = 'Dead-Reck.';
            app.DeadReckButton.Position = [11 34 85 22];

            % Create OdometryButton
            app.OdometryButton = uiradiobutton(app.ModelButtonGroup);
            app.OdometryButton.Text = 'Odometry';
            app.OdometryButton.Position = [11 12 75 22];
            app.OdometryButton.Value = true;

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

            % Create RectengularButton
            app.RectengularButton = uiradiobutton(app.TrajectoryButtonGroup);
            app.RectengularButton.Text = 'Rectengular';
            app.RectengularButton.Position = [11 12 87 22];
            app.RectengularButton.Value = true;

            % Create LandmarkPanel
            app.LandmarkPanel = uipanel(app.UIFigure);
            app.LandmarkPanel.Title = 'Landmark';
            app.LandmarkPanel.Position = [574 288 215 127];

            % Create SigNumEditFieldLabel
            app.SigNumEditFieldLabel = uilabel(app.LandmarkPanel);
            app.SigNumEditFieldLabel.HorizontalAlignment = 'right';
            app.SigNumEditFieldLabel.Position = [21 78 58 22];
            app.SigNumEditFieldLabel.Text = 'Sig. Num.';

            % Create SigNumEditField
            app.SigNumEditField = uieditfield(app.LandmarkPanel, 'numeric');
            app.SigNumEditField.ValueChangedFcn = createCallbackFcn(app, @SigNumEditFieldValueChanged, true);
            app.SigNumEditField.Position = [94 78 100 22];

            % Create SignatureDropDownLabel
            app.SignatureDropDownLabel = uilabel(app.LandmarkPanel);
            app.SignatureDropDownLabel.HorizontalAlignment = 'right';
            app.SignatureDropDownLabel.Position = [25 52 57 22];
            app.SignatureDropDownLabel.Text = 'Signature';

            % Create SignatureDropDown
            app.SignatureDropDown = uidropdown(app.LandmarkPanel);
            app.SignatureDropDown.Items = {'1'};
            app.SignatureDropDown.Position = [97 52 100 22];
            app.SignatureDropDown.Value = '1';

            % Create MxmEditFieldLabel
            app.MxmEditFieldLabel = uilabel(app.LandmarkPanel);
            app.MxmEditFieldLabel.HorizontalAlignment = 'right';
            app.MxmEditFieldLabel.Position = [15 18 43 22];
            app.MxmEditFieldLabel.Text = 'Mx (m)';

            % Create MxmEditField
            app.MxmEditField = uieditfield(app.LandmarkPanel, 'numeric');
            app.MxmEditField.ValueChangedFcn = createCallbackFcn(app, @MxmEditFieldValueChanged, true);
            app.MxmEditField.Position = [73 18 22 22];

            % Create MymEditFieldLabel
            app.MymEditFieldLabel = uilabel(app.LandmarkPanel);
            app.MymEditFieldLabel.HorizontalAlignment = 'right';
            app.MymEditFieldLabel.Position = [114 18 43 22];
            app.MymEditFieldLabel.Text = 'My (m)';

            % Create MymEditField
            app.MymEditField = uieditfield(app.LandmarkPanel, 'numeric');
            app.MymEditField.ValueChangedFcn = createCallbackFcn(app, @MymEditFieldValueChanged, true);
            app.MymEditField.Position = [172 18 22 22];

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
        function app = hw4_exported

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