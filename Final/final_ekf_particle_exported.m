classdef final_ekf_particle_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                     matlab.ui.Figure
        SensorPanel                  matlab.ui.container.Panel
        dmEditFieldLabel             matlab.ui.control.Label
        dmEditField                  matlab.ui.control.NumericEditField
        ThetadgsEditFieldLabel       matlab.ui.control.Label
        ThetadgsEditField            matlab.ui.control.NumericEditField
        ResolutdgsEditFieldLabel     matlab.ui.control.Label
        ResolutdgsEditField          matlab.ui.control.NumericEditField
        sigma_dSliderLabel           matlab.ui.control.Label
        sigma_dSlider                matlab.ui.control.Slider
        sigma_thetSliderLabel        matlab.ui.control.Label
        sigma_thetSlider             matlab.ui.control.Slider
        sigma_sSliderLabel           matlab.ui.control.Label
        sigma_sSlider                matlab.ui.control.Slider
        MainControlPanel             matlab.ui.container.Panel
        ExecuteButton                matlab.ui.control.Button
        ShowScanAreaCheckBox         matlab.ui.control.CheckBox
        UseEKFCheckBox               matlab.ui.control.CheckBox
        UseParticleFilterCheckBox    matlab.ui.control.CheckBox
        SampleNumberEditFieldLabel   matlab.ui.control.Label
        SampleNumberEditField        matlab.ui.control.NumericEditField
        ClearFigureButton            matlab.ui.control.Button
        MotionPanel                  matlab.ui.container.Panel
        alpha1SliderLabel            matlab.ui.control.Label
        alpha1Slider                 matlab.ui.control.Slider
        alpha2SliderLabel            matlab.ui.control.Label
        alpha2Slider                 matlab.ui.control.Slider
        alpha3SliderLabel            matlab.ui.control.Label
        alpha3Slider                 matlab.ui.control.Slider
        alpha4SliderLabel            matlab.ui.control.Label
        alpha4Slider                 matlab.ui.control.Slider
        MethodButtonGroup            matlab.ui.container.ButtonGroup
        DirectEvButton               matlab.ui.control.RadioButton
        SamplingButton               matlab.ui.control.RadioButton
        ModelButtonGroup             matlab.ui.container.ButtonGroup
        DeadReckButton               matlab.ui.control.RadioButton
        OdometryButton               matlab.ui.control.RadioButton
        alpha5SliderLabel            matlab.ui.control.Label
        alpha5Slider                 matlab.ui.control.Slider
        alpha6SliderLabel            matlab.ui.control.Label
        alpha6Slider                 matlab.ui.control.Slider
        TrajectoryButtonGroup        matlab.ui.container.ButtonGroup
        CircularButton               matlab.ui.control.RadioButton
        RectengularButton            matlab.ui.control.RadioButton
        DistrubutionButtonGroup      matlab.ui.container.ButtonGroup
        GaussianButton               matlab.ui.control.RadioButton
        TriangleButton               matlab.ui.control.RadioButton
        ExternalDisturbanceCheckBox  matlab.ui.control.CheckBox
        LandmarkPanel                matlab.ui.container.Panel
        SigNumEditFieldLabel         matlab.ui.control.Label
        SigNumEditField              matlab.ui.control.NumericEditField
        SignatureDropDownLabel       matlab.ui.control.Label
        SignatureDropDown            matlab.ui.control.DropDown
        MxmEditFieldLabel            matlab.ui.control.Label
        MxmEditField                 matlab.ui.control.NumericEditField
        MymEditFieldLabel            matlab.ui.control.Label
        MymEditField                 matlab.ui.control.NumericEditField
        AcceptButton                 matlab.ui.control.Button
        RandomGenButton              matlab.ui.control.Button
        UIAxes                       matlab.ui.control.UIAxes
    end

    
    properties (Access = public)
        sign1 % Description
        sign2
        sign3
        sign4
        sign5
        sign6
        sign7
        sign8
        sign9
        sign10
        sign11
        sign12
        sign13
        sign14
        sign15
        Signs
    end
    
    properties (Access = private)
        Property11 % Description
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
        
        function sample2 = triangle_dist1(app,b)
            ran_num1  = -b + (b+b).*rand(1,1);
            ran_num2  = -b + (b+b).*rand(1,1);
            sample2 = sqrt(6)/2 * (ran_num1 + ran_num2);
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
                
                plot(app.UIAxes,app.sign1.Mx,app.sign1.My,'bo', 'LineWidth', 5);
                hold(app.UIAxes,"on");
                plot(app.UIAxes,app.sign2.Mx,app.sign2.My,'bo', 'LineWidth', 5);
                hold(app.UIAxes, "on");
                dmax = app.dmEditField.Value;
                fimax = deg2rad(app.ThetadgsEditField.Value)/2;
                
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
                        
                        d1 = sqrt((xp(j,i+1) - app.sign1.Mx)^2 + (yp(j,i+1) - app.sign1.My)^2) + normal_dist1(app,app.sigma_dSlider.Value);
                        fi1 = atan2(app.sign1.My-yp(j,i+1), app.sign1.Mx-xp(j,i+1))-tetp(j,i+1) + deg2rad(normal_dist1(app,app.sigma_thetSlider.Value));
                        
                        d2 = sqrt((xp(j,i+1) - app.sign2.Mx)^2 + (yp(j,i+1) - app.sign2.My)^2) + normal_dist1(app,app.sigma_dSlider.Value);
                        fi2 = atan2(app.sign2.My-yp(j,i+1), app.sign2.Mx-xp(j,i+1))-tetp(j,i+1) + deg2rad(normal_dist1(app,app.sigma_thetSlider.Value));
                        
                        see=0;
                        if((d1 <= dmax) && (abs(fi1) <= fimax) )
                            if(app.ShowScanAreaCheckBox.Value == 1)
                                res = app.ResolutdgsEditField.Value ;
                                arr_size = round(2*rad2deg(fimax)/res) + 1;
                                lineptsx = zeros(1,arr_size); lineptsy = zeros(1,arr_size);
                                lineptsx(1) = xp(j,i+1); lineptsy(1) = yp(j,i+1);
                                fi_ray = -fimax;
                                for k=2:arr_size
                                    lineptsx(k) = xp(j,i+1)+dmax*cos(tetp(j,i+1) + fi_ray);
                                    lineptsy(k) = yp(j,i+1)+dmax*sin(tetp(j,i+1) + fi_ray);
                                    fi_ray = fi_ray + deg2rad(res);
                                end
                                %linemaxx = [xp(j,i+1) xp(j,i+1)+dmax*cos(tetp(j,i+1)+fimax) xp(j,i+1)+dmax*cos(tetp(j,i+1)-fimax)];
                                %linemaxy = [yp(j,i+1) yp(j,i+1)+dmax*sin(tetp(j,i+1)+fimax) yp(j,i+1)+dmax*sin(tetp(j,i+1)-fimax)];
                                %pgon = polyshape(linemaxx,linemaxy);
                                pgon = polyshape(lineptsx,lineptsy);
                                plot(app.UIAxes,pgon);
                                hold(app.UIAxes, "on");
                                linex = [xp(j,i+1) app.sign1.Mx];
                                liney = [yp(j,i+1) app.sign1.My];
                                plot(app.UIAxes,linex, liney, '.-r', 'LineWidth', 0.5);
                                hold(app.UIAxes, "on");
                            end
                            plot(app.UIAxes,app.sign1.Mx,app.sign1.My,'bo', 'LineWidth', 1);
                            hold(app.UIAxes,"on");
                            plot(app.UIAxes,app.sign2.Mx,app.sign2.My,'bo', 'LineWidth', 1);
                            hold(app.UIAxes, "on");
                            plot(app.UIAxes, xp(j,i+1)+d1*cos(tetp(j,i+1)+fi1), yp(j,i+1)+d1*sin(tetp(j,i+1)+fi1), 'r.','Linewidth',2);
                            hold(app.UIAxes, "on");
                            plot(app.UIAxes,xp(j,i+1),yp(j,i+1),'g.');
                            hold(app.UIAxes, "on");
                            see=1;
                        end
                        
                        if((d2 <= dmax) && (abs(fi2) <= fimax) )
                            if(app.ShowScanAreaCheckBox.Value == 1)
                                res = app.ResolutdgsEditField.Value ;
                                arr_size = round(2*rad2deg(fimax)/res) + 1;
                                lineptsx = zeros(1,arr_size); lineptsy = zeros(1,arr_size);
                                lineptsx(1) = xp(j,i+1); lineptsy(1) = yp(j,i+1);
                                fi_ray = -fimax;
                                for k=2:arr_size
                                    lineptsx(k) = xp(j,i+1)+dmax*cos(tetp(j,i+1) + fi_ray);
                                    lineptsy(k) = yp(j,i+1)+dmax*sin(tetp(j,i+1) + fi_ray);
                                    fi_ray = fi_ray + deg2rad(res);
                                end
                                linemaxx = [xp(j,i+1) xp(j,i+1)+dmax*cos(tetp(j,i+1)+fimax) xp(j,i+1)+dmax*cos(tetp(j,i+1)-fimax)];
                                linemaxy = [yp(j,i+1) yp(j,i+1)+dmax*sin(tetp(j,i+1)+fimax) yp(j,i+1)+dmax*sin(tetp(j,i+1)-fimax)];
                                pgon = polyshape(lineptsx,lineptsy);
                                plot(app.UIAxes,pgon);
                                hold(app.UIAxes, "on");
                                linex = [xp(j,i+1) app.sign2.Mx];
                                liney = [yp(j,i+1) app.sign2.My];
                                plot(app.UIAxes,linex, liney, '.-r', 'LineWidth', 0.5);
                                hold(app.UIAxes, "on");
                            end
                            plot(app.UIAxes,app.sign1.Mx,app.sign1.My,'bo', 'LineWidth', 1);
                            hold(app.UIAxes,"on");
                            plot(app.UIAxes,app.sign2.Mx,app.sign2.My,'bo', 'LineWidth', 1);
                            hold(app.UIAxes, "on");
                            
                            plot(app.UIAxes, xp(j,i+1)+d2*cos(tetp(j,i+1)+fi2), yp(j,i+1)+d2*sin(tetp(j,i+1)+fi2), 'r.', 'Linewidth',2);
                            hold(app.UIAxes, "on");
                            plot(app.UIAxes,xp(j,i+1),yp(j,i+1),'g.');
                            hold(app.UIAxes, "on");
                            see=1;
                        end
                        
                        if(~see)
                            plot(app.UIAxes,xp(j,i+1),yp(j,i+1),'k.');
                            hold(app.UIAxes, "on");
                        end
                        
                    end
                end
                % plot(app.UIAxes,xp,yp,'k.');
                % hold(app.UIAxes,"on");
                hold(app.UIAxes, "off");
                
            end
        end
        
        function dead_reckoning(app)
            a1 = app.alpha1Slider.Value;  a2 = app.alpha2Slider.Value;
            a3 = app.alpha3Slider.Value;  a4 = app.alpha4Slider.Value;
            a5 = app.alpha5Slider.Value;  a6 = app.alpha6Slider.Value;
            if(app.CircularButton.Value == 1)
                sample_n = 100;
                x=zeros(sample_n,8); y=zeros(sample_n,8); tet = zeros(sample_n,8);
                xm=zeros(sample_n,8); ym=zeros(sample_n,8); tetm = zeros(sample_n,8);
                x(:,1) = 4; tet(:,1) = pi/2;
                xm(:,1) = 4; tetm(:,1) = pi/2; %Initial position.
                
                
                
                xekf=zeros(1,150); yekf=zeros(1,150);
                xblack=zeros(1,150); yblack=zeros(1,150);
                ekf_c = 1; black_c=1;
                zt1s=zeros(3,1); zt2s=zeros(3,1); zt1=zeros(3,1); zt2=zeros(3,1);
                Kt1=zeros(3); Kt2=zeros(1,3); Ht1 = zeros(3);
                sn = app.SigNumEditField.Value;
                z=zeros(sn,3); zs=zeros(sn,3);
                
                r = 4;
                t = 1; %time of traviling for 45 degrees
                w = -pi/10; %Assumed 45 degrees travel in one sec.
                v = w * r;
                
                s1=a1*v^2+a2*w^2; s2=a3*v^2+a4*w^2; s3=a5*v^2+a6*w^2;
                Stp = zeros(3); St_= zeros(3); St_(1,1) = s1; St_(2,2) = s2; St_(3,3) = s3;
                plot(app.UIAxes,x(1,1),y(1,1),'k.','LineWidth',2);
                hold(app.UIAxes,"on");
                dmax = app.dmEditField.Value;
                fimax = deg2rad(app.ThetadgsEditField.Value)/2;
                for i=1:20
                    for j=1:sample_n
                        v_hat = v + normal_dist1(app,s1);
                        w_hat = w + normal_dist1(app,s2);
                        gama_hat = normal_dist1(app,s3);
                        
                        %Actual states
                        x(j,i+1) = x(j,i) -  v_hat/w_hat * sin(tet(j,i)) +  v_hat/w_hat * sin(tet(j,i) + w_hat*t);
                        y(j,i+1) = y(j,i) +  v_hat/w_hat * cos(tet(j,i)) -  v_hat/w_hat * cos(tet(j,i) + w_hat*t);
                        tet(j,i+1) = tet(j,i) + w_hat*t + gama_hat*t;
                        X = [x(j,i+1); y(j,i+1); tet(j,i+1)];
                        
                        %Predicted states
                        if(abs(w)>0.1)
                            xm(j,i+1) = xm(j,i) -  v/w * sin(tetm(j,i)) +  v/w * sin(tetm(j,i) + w*t);
                            ym(j,i+1) = ym(j,i) +  v/w * cos(tetm(j,i)) -  v/w * cos(tetm(j,i) + w*t);
                            tetm(j,i+1) = tetm(j,i) + w*t;
                        else
                            xm(j,i+1) = xm(j,i) +  v * cos(tetm(j,i));
                            ym(j,i+1) = ym(j,i) +  v * sin(tetm(j,i));
                            tetm(j,i+1) = tetm(j,i);
                        end
                        
                        Xm = [xm(j,i+1); ym(j,i+1); tetm(j,i+1)];
                        
                        
                        for m=1:app.SigNumEditField.Value
                            %Actual measurements
                            z(m,1) = sqrt((x(j,i+1) - app.Signs(m).Mx)^2 + (y(j,i+1) - app.Signs(m).My)^2) + normal_dist1(app,app.sigma_dSlider.Value);
                            z(m,2) = atan2(app.Signs(m).My-y(j,i+1), app.Signs(m).Mx-x(j,i+1))-tet(j,i+1) + deg2rad(normal_dist1(app,app.sigma_thetSlider.Value));
                            
                            %Supposed measuremets
                            zs(m,1) = sqrt((xm(j,i+1) - app.Signs(m).Mx)^2 + (ym(j,i+1) - app.Signs(m).My)^2);
                            zs(m,2) = atan2(app.Signs(m).My-ym(j,i+1), app.Signs(m).Mx-xm(j,i+1))-tetm(j,i+1);
                            plot(app.UIAxes,app.Signs(m).Mx,app.Signs(m).My,'bo', 'LineWidth', 1);
                            hold(app.UIAxes,"on");
                        end
                        
                        %                 d2 = sqrt((x(j,i+1) - app.sign2.Mx)^2 + (y(j,i+1) - app.sign2.My)^2) + normal_dist1(app,app.sigma_dSlider.Value);
                        %                 fi2 = atan2(app.sign2.My-y(j,i+1), app.sign2.Mx-x(j,i+1))-tet(j,i+1) + deg2rad(normal_dist1(app,app.sigma_thetSlider.Value));
                        %                 zt2 = [d2; fi2; 1];
                        %
                        %
                        
                        %
                        %                 d2s = sqrt((xm(j,i+1) - app.sign2.Mx)^2 + (ym(j,i+1) - app.sign2.My)^2);
                        %                 fi2s = atan2(app.sign2.My-ym(j,i+1), app.sign2.Mx-xm(j,i+1))-tetm(j,i+1);
                        %                 zt2s = [d2s; fi2s; 2];
                        
                        
                        
                        
                        see=0;
                        if(app.UseEKFCheckBox.Value == 1)
                            Gt = ling(app,v,w,tet(j,i));
                            Ct = linc(app,v,w,tet(j,i));
                            Su = covu(app,a1,a2,a3,a4,a5,a6,v,w) ;
                            St_ = transpose(Gt)*St_*Gt+transpose(Ct)*Su*Ct;
                            
                        end
                        
                        for m=1:app.SigNumEditField.Value
                            if((z(m,1) <= dmax) && (abs(z(m,2)) <= fimax) )      % landmark has been seen
                                Ht1 = linh(app,xm(j,i+1),ym(j,i+1),app.Signs(m).Mx,app.Signs(m).My,zs(m,1)) ;
                                zt1 = [z(m,1);z(m,2);0];
                                zt1s = [zs(m,1);zs(m,2);0];
                                if(app.UseEKFCheckBox.Value == 1)
                                    Em = [app.sigma_dSlider.Value^2 0 0; 0 app.sigma_thetSlider.Value^2 0; 0 0 0.01];
                                    St1 = Ht1*St_*transpose(Ht1)+Em;
                                    Kt1 = St_*transpose(Ht1)*inv(St1);
                                    
                                end
                                
                                if(app.UseEKFCheckBox.Value == 1)
                                    % fprintf("Error:")
                                    %(zt1-zt1s)
                                    C = Kt1*(zt1-zt1s);
                                    %fprintf("Before filter:");
                                    % x(j,i+1)
                                    xm(j,i+1) = Xm(1) + C(1);
                                    ym(j,i+1) = Xm(2) + C(2);
                                    tetm(j,i+1) = Xm(3) + C(3);
                                    x(j,i+1) = xm(j,i+1);
                                    y(j,i+1)= ym(j,i+1);
                                    tet(j,i+1) = tetm(j,i+1);
                                    %fprintf("After filter:");
                                    % x(j,i+1)
                                    St_ = (eye(3) - (Kt1*Ht1)) * St_;
                                    %s1 = St_(1,1); s2 = St_(2,2); s3 = St_(3,3);
                                end
                                
                                xekf(ekf_c) = x(j,i+1);
                                yekf(ekf_c) = y(j,i+1);
                                ekf_c=ekf_c+1;
                                
                                if(app.ShowScanAreaCheckBox.Value == 1)
                                    res = app.ResolutdgsEditField.Value ;
                                    arr_size = round(2*rad2deg(fimax)/res) + 1;
                                    lineptsx = zeros(1,arr_size); lineptsy = zeros(1,arr_size);
                                    lineptsx(1) = x(j,i+1); lineptsy(1) = y(j,i+1);
                                    fi_ray = -fimax;
                                    for k=2:arr_size
                                        lineptsx(k) = x(j,i+1)+dmax*cos(tet(j,i+1) + fi_ray);
                                        lineptsy(k) = y(j,i+1)+dmax*sin(tet(j,i+1) + fi_ray);
                                        fi_ray = fi_ray + deg2rad(res);
                                    end
                                    %linemaxx = [xp(j,i+1) xp(j,i+1)+dmax*cos(tetp(j,i+1)+fimax) xp(j,i+1)+dmax*cos(tetp(j,i+1)-fimax)];
                                    %linemaxy = [yp(j,i+1) yp(j,i+1)+dmax*sin(tetp(j,i+1)+fimax) yp(j,i+1)+dmax*sin(tetp(j,i+1)-fimax)];
                                    %pgon = polyshape(linemaxx,linemaxy);
                                    pgon = polyshape(lineptsx,lineptsy);
                                    plot(app.UIAxes,pgon);
                                    hold(app.UIAxes, "on");
                                    linex = [x(j,i+1) app.Signs(m).Mx];
                                    liney = [y(j,i+1) app.Signs(m).My];
                                    plot(app.UIAxes,linex, liney, '.-r', 'LineWidth', 0.5);
                                    hold(app.UIAxes, "on");
                                end
                                
                                %                                 plot(app.UIAxes, x(j,i+1)+z(m,1)*cos(tet(j,i+1)+z(m,2)), y(j,i+1)+z(m,1)*sin(tet(j,i+1)+z(m,2)), 'r.','Linewidth',2);
                                %                                 hold(app.UIAxes, "on");
                                see=1;
                                %                                 plot(app.UIAxes,x(j,i+1),y(j,i+1),'g.','LineWidth',1);
                                %                                 hold(app.UIAxes, "on");
                                
                            else
                                see=0;
                                xblack(black_c) = x(j,i+1);
                                yblack(black_c) = y(j,i+1);
                                black_c = black_c + 1;
                                %                                 plot(app.UIAxes,x(j,i+1),y(j,i+1),'k.');
                                %                                 hold(app.UIAxes, "on");
                            end
                            %X
                            %                             plot(app.UIAxes,app.Signs(m).Mx,app.Signs(m).My,'bo', 'LineWidth', 1);
                            %                             hold(app.UIAxes,"on");
                        end
                        
                        
                        
                        
                        
                        
                    end
                end
                
                plot(app.UIAxes,xblack,yblack,'k.','LineWidth',1);
                hold(app.UIAxes,'on');
                plot(app.UIAxes,xekf,yekf,'g.','LineWidth',1);
                hold(app.UIAxes,'off');
            end
            
            if(app.RectengularButton.Value == 1)
                sample_n = app.SampleNumberEditField.Value;
                x=zeros(sample_n,13); y=zeros(sample_n,13); tet = zeros(sample_n,13);
                xm=zeros(sample_n,13); ym=zeros(sample_n,13); tetm = zeros(sample_n,13);
                X_w = zeros(sample_n, 4); %Particle filter stata and weights.
                x(:,1) = -4; y(:,1) = -4; tet(:,1) = 0; %Initial position.
                xm(:,1) = -4; ym(:,1) = -4; tetm(:,1) = 0; %Initial position.
                xekf=zeros(1,150); yekf=zeros(1,150);
                x_part = zeros(1,sample_n*13); y_part = zeros(1,sample_n*13); tet_part = zeros(1,sample_n*13);
                xblack=zeros(1,150); yblack=zeros(1,150);
                ekf_c = 1; black_c=1; p_c = 1;
                zt1s=zeros(3,1); zt2s=zeros(3,1); zt1=zeros(3,1); zt2=zeros(3,1);
                Kt1=zeros(3); Kt2=zeros(1,3); Ht1 = zeros(3);
                sn = app.SigNumEditField.Value;
                z=zeros(sn,3); z_mu=zeros(sn,3); zs=zeros(sn,3);
                t = 1; %time of traviling for 2 meters
                v = 2; %Assumed traveled 2 meters in 1 sec.
                w = 0.01; %Assumed no rotation.
                we = 1/sample_n;
                vd = 1;
                
                s1=a1*v^2+a2*w^2; s2=a3*v^2+a4*w^2; s3=a5*v^2+a6*w^2;
                Stp = zeros(3); St_= zeros(3);
                plot(app.UIAxes,x(1,1),y(1,1),'k.','LineWidth',2);
                hold(app.UIAxes,"on");
                dmax = app.dmEditField.Value;
                fimax = deg2rad(app.ThetadgsEditField.Value)/2;
                for i=1:12
                    if(i == 4 || i == 8)
                        w = pi/2;
                    else
                        w = 0.001;
                    end
                    
                    
                    % plot(app.UIAxes,xm(i+1) ,ym(i+1),'go', 'LineWidth', 5);
                    %  hold(app.UIAxes,"on");
                    
                    for j=1:sample_n
                        if(app.GaussianButton.Value == 1)
                            v_hat = v + normal_dist1(app,s1);
                            w_hat = w + normal_dist1(app,s2);
                            gama_hat = normal_dist1(app,s3);
                        else
                            v_hat = v + triangle_dist1(app,s1);
                            w_hat = w + triangle_dist1(app,s2);
                            gama_hat = triangle_dist1(app,s3);
                        end
                        
                        if(app.ExternalDisturbanceCheckBox.Value == 1)
                            if(i == 6)
                                v_hat = v_hat + vd;
                            end
                        end
                        
                        %Actual states
                        x(j,i+1) = x(j,i) -  v_hat/w_hat * sin(tet(j,i)) +  v_hat/w_hat * sin(tet(j,i) + w_hat*t);
                        y(j,i+1) = y(j,i) +  v_hat/w_hat * cos(tet(j,i)) -  v_hat/w_hat * cos(tet(j,i) + w_hat*t);
                        tet(j,i+1) = tet(j,i) + w_hat*t + gama_hat*t;
                        X = [x(j,i+1); y(j,i+1); tet(j,i+1)];
                        X_w(j,:) = [X' we]; %Particle filter state
                        
                        %Predicted states
                        if(abs(w)>1)
                            xm(j,i+1) = xm(j,i) -  v/w * sin(tetm(j,i)) +  v/w * sin(tetm(j,i) + w*t);
                            ym(j,i+1) = ym(j,i) +  v/w * cos(tetm(j,i)) -  v/w * cos(tetm(j,i) + w*t);
                            tetm(j,i+1) = tetm(j,i) + w*t;
                        else
                            xm(j,i+1) = xm(j,i) +  v * cos(tetm(j,i));
                            ym(j,i+1) = ym(j,i) +  v * sin(tetm(j,i));
                            tetm(j,i+1) = tetm(j,i);
                        end
                        
                        Xm = [xm(j,i+1); ym(j,i+1); tetm(j,i+1)];
                        
                        
                        for m=1:app.SigNumEditField.Value
                            %Mean value measurements
                            z_mu(m,1) = sqrt((xm(j,i+1) - app.Signs(m).Mx)^2 + (ym(j,i+1) - app.Signs(m).My)^2);
                            z_mu(m,2) = atan2(app.Signs(m).My-ym(j,i+1), app.Signs(m).Mx-xm(j,i+1))-tetm(j,i+1);
                            
                            %Actual measurements
                            z(m,1) = sqrt((x(j,i+1) - app.Signs(m).Mx)^2 + (y(j,i+1) - app.Signs(m).My)^2) + normal_dist1(app,app.sigma_dSlider.Value);
                            z(m,2) = atan2(app.Signs(m).My-y(j,i+1), app.Signs(m).Mx-x(j,i+1))-tet(j,i+1) + deg2rad(normal_dist1(app,app.sigma_thetSlider.Value));
                            
                            %Supposed measuremets
                            zs(m,1) = sqrt((xm(j,i+1) - app.Signs(m).Mx)^2 + (ym(j,i+1) - app.Signs(m).My)^2);
                            zs(m,2) = atan2(app.Signs(m).My-ym(j,i+1), app.Signs(m).Mx-xm(j,i+1))-tetm(j,i+1);
                            
                            
                            
                            plot(app.UIAxes,app.Signs(m).Mx,app.Signs(m).My,'bo', 'LineWidth', 5);
                            hold(app.UIAxes,"on");
                        end
                        
                        
                        see=0;
                        if(app.UseEKFCheckBox.Value == 1)
                            Gt = ling(app,v,w,tetm(j,i));
                            Ct = linc(app,v,w,tetm(j,i));
                            Su = covu(app,a1,a2,a3,a4,a5,a6,v,w) ;
                            St_ = Gt*St_*transpose(Gt)+Ct*Su*transpose(Ct);
                            
                        end
                        
                        for m=1:app.SigNumEditField.Value
                            if((z(m,1) <= dmax) && (abs(z(m,2)) <= fimax) )      % landmark has been seen
                                Ht1 = linh(app,xm(j,i+1),ym(j,i+1),app.Signs(m).Mx,app.Signs(m).My,zs(m,1)) ;
                                zt1 = [z(m,1);z(m,2);0];
                                zt1s = [zs(m,1);zs(m,2);0];
                                
                                if(app.UseEKFCheckBox.Value == 1)
                                    
                                    %                                     Gt = ling(app,v,w,tet(j,i));
                                    %                                     Ct = linc(app,v,w,tet(j,i));
                                    %                                     Su = covu(app,a1,a2,a3,a4,a5,a6,v,w) ;
                                    %                                     St_ = transpose(Gt)*St_*Gt+transpose(Ct)*Su*Ct;
                                    
                                    
                                    Em = [app.sigma_dSlider.Value^2 0 0; 0 app.sigma_thetSlider.Value^2 0; 0 0 0.01];
                                    St1 = Ht1*St_*transpose(Ht1)+Em;
                                    Kt1 = St_*transpose(Ht1)/(St1);
                                    
                                end
                                
                                %Calculate measurement probabilities
                                if(app.UseParticleFilterCheckBox.Value == 1)
                                    we = prob(app,z_mu(m,1)-z(m,1),app.sigma_dSlider.Value) * prob(app,z_mu(m,2)-z(m,2),app.sigma_thetSlider.Value);
                                    X_w(j,:) = [X' we]; %Particle filter state
                                end
                                
                                
                                
                                if(app.UseEKFCheckBox.Value == 1)
                                    % fprintf("Error:")
                                  
                                    C = Kt1*(zt1-zt1s);
                                    %fprintf("Before filter:");
                                    % x(j,i+1)

                                    x(j,i+1) = Xm(1) + C(1);
                                    y(j,i+1)  = Xm(2) + C(2);
                                    tet(j,i+1) = Xm(3) + C(3);
%                                     x(j,i+1) = xm(j,i+1);
%                                     y(j,i+1) = ym(j,i+1);
%                                     tet(j,i+1) = tetm(j,i+1);
                                    
                                    %fprintf("After filter:");
                                    % x(j,i+1)
                                    St_ = (eye(3) - (Kt1*Ht1)) * St_;
                                    %s1 = St_(1,1); s2 = St_(2,2); s3 = St_(3,3);
                                end
                                
                                xekf(ekf_c) = x(j,i+1);
                                yekf(ekf_c) = y(j,i+1);
                                ekf_c=ekf_c+1;
                                
                                if(app.ShowScanAreaCheckBox.Value == 1)
                                    res = app.ResolutdgsEditField.Value ;
                                    arr_size = round(2*rad2deg(fimax)/res) + 1;
                                    lineptsx = zeros(1,arr_size); lineptsy = zeros(1,arr_size);
                                    lineptsx(1) = x(j,i+1); lineptsy(1) = y(j,i+1);
                                    fi_ray = -fimax;
                                    for k=2:arr_size
                                        lineptsx(k) = x(j,i+1)+dmax*cos(tet(j,i+1) + fi_ray);
                                        lineptsy(k) = y(j,i+1)+dmax*sin(tet(j,i+1) + fi_ray);
                                        fi_ray = fi_ray + deg2rad(res);
                                    end
                                    %linemaxx = [xp(j,i+1) xp(j,i+1)+dmax*cos(tetp(j,i+1)+fimax) xp(j,i+1)+dmax*cos(tetp(j,i+1)-fimax)];
                                    %linemaxy = [yp(j,i+1) yp(j,i+1)+dmax*sin(tetp(j,i+1)+fimax) yp(j,i+1)+dmax*sin(tetp(j,i+1)-fimax)];
                                    %pgon = polyshape(linemaxx,linemaxy);
                                    pgon = polyshape(lineptsx,lineptsy);
                                    plot(app.UIAxes,pgon);
                                    hold(app.UIAxes, "on");
                                    linex = [x(j,i+1) app.Signs(m).Mx];
                                    liney = [y(j,i+1) app.Signs(m).My];
                                    %                                     plot(app.UIAxes,linex, liney, '.-r', 'LineWidth', 0.5);
                                    %                                     hold(app.UIAxes, "on");
                                end
                                
                                %                                 plot(app.UIAxes, x(j,i+1)+z(m,1)*cos(tet(j,i+1)+z(m,2)), y(j,i+1)+z(m,1)*sin(tet(j,i+1)+z(m,2)), 'r.','Linewidth',2);
                                %                                 hold(app.UIAxes, "on");
                                see=1;
                                %                                 plot(app.UIAxes,x(j,i+1),y(j,i+1),'g.','LineWidth',1);
                                %                                 hold(app.UIAxes, "on");
                                
                            else
                                see=0;
                                xblack(black_c) = x(j,i+1);
                                yblack(black_c) = y(j,i+1);
                                black_c = black_c + 1;
                                %                                 plot(app.UIAxes,x(j,i+1),y(j,i+1),'k.');
                                %                                 hold(app.UIAxes, "on");
                            end
                            %X
                            %                             plot(app.UIAxes,app.Signs(m).Mx,app.Signs(m).My,'bo', 'LineWidth', 1);
                            %                             hold(app.UIAxes,"on");
                        end
                        
                        
                    end
                    
                    if(app.UseParticleFilterCheckBox.Value == 0)
                        if(app.UseEKFCheckBox.Value == 1)
                            uncertainty_ellipse(x(:,i+1), y(:,i+1),'blue')
                        else
                            uncertainty_ellipse(x(:,i+1), y(:,i+1),'black')
                        end
                        hold on;
                        
                        xreal = xm(:,i+1)
                    end
                    
                    if(app.UseParticleFilterCheckBox.Value == 1)
                        R = randp(X_w(:,4)', 1, sample_n);
                        perc=histc(R, 1:sample_n); %Check probabilities for debugging.
                        for j=1:sample_n
                            x_part(p_c) = X_w(R(j),1);
                            y_part(p_c) = X_w(R(j),2);
                            tet_part(p_c) = X_w(R(j),3);
                            
                            %                             xm(j,i+1) = x_part(p_c);
                            %                             ym(j,i+1) = y_part(p_c);
                            %                             tetm(j,i+1) = tet_part(p_c);
                            
                            x(j,i+1) = x_part(p_c);
                            y(j,i+1) = y_part(p_c);
                            tet(j,i+1) = tet_part(p_c);
                            p_c = p_c + 1;
                        end
                        uncertainty_ellipse(x(:,i+1), y(:,i+1),'red')
                        hold on;
                    end
                end
                
                
                
                
                
                
                if(app.UseEKFCheckBox.Value == 1)
                    plot(app.UIAxes,xblack,yblack,'.','Color','#D95319','LineWidth',1);
                    hold(app.UIAxes,'on');
                    plot(app.UIAxes,xekf,yekf,'.','Color','#D95319','LineWidth',1);
                    % hold(app.UIAxes,'off');
                    
                    
                elseif(app.UseParticleFilterCheckBox.Value == 1)
                    plot(app.UIAxes,xblack,yblack,'g.','LineWidth',1);
                    hold(app.UIAxes,'on');
                    plot(app.UIAxes,x_part,y_part,'g.','LineWidth',1);
                    %hold(app.UIAxes,'off');
                else
                    plot(app.UIAxes,xblack,yblack,'k.','LineWidth',1);
                    hold(app.UIAxes,'on');
                end
                
                
            end
            %x
            %ym
        end
        
        function Gt = ling(~,v,w,tet)
            a = v/w;
            if(abs(w)>0.1)
                Gt = [1 0 -a*cos(tet)+a*cos(tet+w);...
                    0 1 -a*sin(tet)+a*sin(tet+w);...
                    0 0 1];
            else
                Gt=[1 0 -v*sin(tet); 0 1 v*cos(tet); 0 0 1];
                %fprintf("Linear mod.\n");
            end
        end
        
        function Ct = linc(~,v,w,tet)
            a = v/w;
            if(abs(w)>0.1)
                Ct = 1/w * [-sin(tet)+sin(tet+w) a*(sin(tet)-sin(tet+w))+v*cos(tet+w) 0;...
                    cos(tet)-cos(tet+w)  a*(-cos(tet)+cos(tet+w))+v*sin(tet+w) 0;...
                    0 0 w];
            else
                Ct = [cos(tet) 0 0; sin(tet) 0 0; 0 0 0];
            end
        end
        function Ht = linh(~,x,y,mx,my,d)
            Ht = [-(mx - x)/d -(my-y)/d  0;...
                (my - y)/d^2  -(mx-x)/d^2 -1;...
                0           0          0];
        end
        
        
        function sigmau = covu(~,a1,a2,a3,a4,a5,a6,v,w)
            sigmau = [a1*v^2+a2*w^2 0             0;...
                0             a3*v^2+a4*w^2 0;...
                0             0             a5*v^2+a6*w^2];
        end
        
        
        function gauss = prob(~, a,b)
            gauss = (1/sqrt(2*pi*b^2))*exp(-0.5*(a^2)/(b^2));
        end
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.sign1 = Signature;
            app.sign2 = Signature;
            app.sign3 = Signature;
            app.sign4 = Signature;
            app.sign5 = Signature;
            app.sign6 = Signature;
            app.sign7 = Signature;
            app.sign8 = Signature;
            app.sign9 = Signature;
            app.sign10 = Signature;
            app.sign11 = Signature;
            app.sign12 = Signature;
            app.sign13 = Signature;
            app.sign14 = Signature;
            app.sign15 = Signature;
            app.sign1.Mx = 1; app.sign1.My = -4;
            app.sign2.Mx = 3.2; app.sign2.My = 0;
            app.sign3.Mx = 1.8; app.sign3.My = 6;
            
            app.Signs = [app.sign1 app.sign2 app.sign3 app.sign4 app.sign5...
                app.sign6 app.sign7 app.sign8 app.sign9 app.sign10 app.sign11 app.sign12 app.sign13 app.sign14 app.sign15];
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

        % Button pushed function: AcceptButton
        function AcceptButtonPushed(app, event)
            mx = app.MxmEditField.Value;
            my = app.MymEditField.Value;
            if(app.SignatureDropDown.Value == "1")
                app.sign1.Mx = mx;
                app.sign1.My = my;
                plot(app.UIAxes,app.sign1.Mx,app.sign1.My,'bo', 'LineWidth', 10);
                hold(app.UIAxes,"on");
                
                
            elseif(app.SignatureDropDown.Value == "2")
                app.sign2.Mx = mx;
                app.sign2.My = my;
                plot(app.UIAxes,app.sign2.Mx,app.sign2.My,'bo', 'LineWidth', 10);
                hold(app.UIAxes,"on");
                
                
            elseif(app.SignatureDropDown.Value == "3")
                app.sign3.Mx = mx;
                app.sign3.My = my;
                plot(app.UIAxes,app.sign3.Mx,app.sign3.My,'bo', 'LineWidth', 10);
                hold(app.UIAxes,"on");
            end
        end

        % Button pushed function: RandomGenButton
        function RandomGenButtonPushed(app, event)
            for i=1:app.SigNumEditField.Value
                app.Signs(i).Mx = randi([-6 6],1,1); app.Signs(i).My = randi([-6 6],1,1);
                plot(app.UIAxes,app.Signs(i).Mx,app.Signs(i).My,'bo', 'LineWidth', 10);
                hold(app.UIAxes,"on");
                plot(app.UIAxes,app.Signs(i).Mx,app.Signs(i).My,'bo', 'LineWidth', 10);
            end
            hold(app.UIAxes,"off");
        end

        % Button pushed function: ClearFigureButton
        function ClearFigureButtonPushed(app, event)
            cla(app.UIAxes);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 814 728];
            app.UIFigure.Name = 'MATLAB App';

            % Create SensorPanel
            app.SensorPanel = uipanel(app.UIFigure);
            app.SensorPanel.Title = 'Sensor';
            app.SensorPanel.Position = [574 451 215 250];

            % Create dmEditFieldLabel
            app.dmEditFieldLabel = uilabel(app.SensorPanel);
            app.dmEditFieldLabel.HorizontalAlignment = 'right';
            app.dmEditFieldLabel.Position = [42 197 37 22];
            app.dmEditFieldLabel.Text = 'd (m) ';

            % Create dmEditField
            app.dmEditField = uieditfield(app.SensorPanel, 'numeric');
            app.dmEditField.Position = [94 197 100 22];
            app.dmEditField.Value = 4;

            % Create ThetadgsEditFieldLabel
            app.ThetadgsEditFieldLabel = uilabel(app.SensorPanel);
            app.ThetadgsEditFieldLabel.HorizontalAlignment = 'right';
            app.ThetadgsEditFieldLabel.Position = [15 167 67 22];
            app.ThetadgsEditFieldLabel.Text = 'Theta (dgs)';

            % Create ThetadgsEditField
            app.ThetadgsEditField = uieditfield(app.SensorPanel, 'numeric');
            app.ThetadgsEditField.Position = [97 167 100 22];
            app.ThetadgsEditField.Value = 90;

            % Create ResolutdgsEditFieldLabel
            app.ResolutdgsEditFieldLabel = uilabel(app.SensorPanel);
            app.ResolutdgsEditFieldLabel.HorizontalAlignment = 'right';
            app.ResolutdgsEditFieldLabel.Position = [-1 134 80 22];
            app.ResolutdgsEditFieldLabel.Text = 'Resolut. (dgs)';

            % Create ResolutdgsEditField
            app.ResolutdgsEditField = uieditfield(app.SensorPanel, 'numeric');
            app.ResolutdgsEditField.Position = [94 134 100 22];
            app.ResolutdgsEditField.Value = 2;

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
            app.sigma_dSlider.Value = 0.03;

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
            app.sigma_thetSlider.Value = 0.03;

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

            % Create MainControlPanel
            app.MainControlPanel = uipanel(app.UIFigure);
            app.MainControlPanel.Title = 'Main Control';
            app.MainControlPanel.Position = [573 18 214 245];

            % Create ExecuteButton
            app.ExecuteButton = uibutton(app.MainControlPanel, 'push');
            app.ExecuteButton.ButtonPushedFcn = createCallbackFcn(app, @ExecuteButtonPushed, true);
            app.ExecuteButton.Position = [57 50 100 22];
            app.ExecuteButton.Text = 'Execute';

            % Create ShowScanAreaCheckBox
            app.ShowScanAreaCheckBox = uicheckbox(app.MainControlPanel);
            app.ShowScanAreaCheckBox.Text = 'Show Scan Area';
            app.ShowScanAreaCheckBox.Position = [50 151 111 22];

            % Create UseEKFCheckBox
            app.UseEKFCheckBox = uicheckbox(app.MainControlPanel);
            app.UseEKFCheckBox.Text = 'Use EKF';
            app.UseEKFCheckBox.Position = [50 119 70 22];

            % Create UseParticleFilterCheckBox
            app.UseParticleFilterCheckBox = uicheckbox(app.MainControlPanel);
            app.UseParticleFilterCheckBox.Text = 'Use Particle Filter';
            app.UseParticleFilterCheckBox.Position = [50 84 117 22];

            % Create SampleNumberEditFieldLabel
            app.SampleNumberEditFieldLabel = uilabel(app.MainControlPanel);
            app.SampleNumberEditFieldLabel.HorizontalAlignment = 'right';
            app.SampleNumberEditFieldLabel.Position = [27 186 92 22];
            app.SampleNumberEditFieldLabel.Text = 'Sample Number';

            % Create SampleNumberEditField
            app.SampleNumberEditField = uieditfield(app.MainControlPanel, 'numeric');
            app.SampleNumberEditField.Position = [132 186 53 22];
            app.SampleNumberEditField.Value = 250;

            % Create ClearFigureButton
            app.ClearFigureButton = uibutton(app.MainControlPanel, 'push');
            app.ClearFigureButton.ButtonPushedFcn = createCallbackFcn(app, @ClearFigureButtonPushed, true);
            app.ClearFigureButton.Position = [57 12 100 22];
            app.ClearFigureButton.Text = 'Clear Figure';

            % Create MotionPanel
            app.MotionPanel = uipanel(app.UIFigure);
            app.MotionPanel.Title = 'Motion';
            app.MotionPanel.Position = [22 29 524 306];

            % Create alpha1SliderLabel
            app.alpha1SliderLabel = uilabel(app.MotionPanel);
            app.alpha1SliderLabel.HorizontalAlignment = 'right';
            app.alpha1SliderLabel.FontSize = 10;
            app.alpha1SliderLabel.Position = [15 245 35 22];
            app.alpha1SliderLabel.Text = 'alpha1';

            % Create alpha1Slider
            app.alpha1Slider = uislider(app.MotionPanel);
            app.alpha1Slider.Limits = [0 0.05];
            app.alpha1Slider.FontSize = 10;
            app.alpha1Slider.Position = [71 254 110 3];
            app.alpha1Slider.Value = 0.005;

            % Create alpha2SliderLabel
            app.alpha2SliderLabel = uilabel(app.MotionPanel);
            app.alpha2SliderLabel.HorizontalAlignment = 'right';
            app.alpha2SliderLabel.FontSize = 10;
            app.alpha2SliderLabel.Position = [200 243 35 22];
            app.alpha2SliderLabel.Text = 'alpha2';

            % Create alpha2Slider
            app.alpha2Slider = uislider(app.MotionPanel);
            app.alpha2Slider.Limits = [0 0.05];
            app.alpha2Slider.FontSize = 10;
            app.alpha2Slider.Position = [256 252 110 3];
            app.alpha2Slider.Value = 0.005;

            % Create alpha3SliderLabel
            app.alpha3SliderLabel = uilabel(app.MotionPanel);
            app.alpha3SliderLabel.HorizontalAlignment = 'right';
            app.alpha3SliderLabel.FontSize = 10;
            app.alpha3SliderLabel.Position = [15 185 35 22];
            app.alpha3SliderLabel.Text = 'alpha3';

            % Create alpha3Slider
            app.alpha3Slider = uislider(app.MotionPanel);
            app.alpha3Slider.Limits = [0 0.05];
            app.alpha3Slider.FontSize = 10;
            app.alpha3Slider.Position = [71 194 110 3];
            app.alpha3Slider.Value = 0.005;

            % Create alpha4SliderLabel
            app.alpha4SliderLabel = uilabel(app.MotionPanel);
            app.alpha4SliderLabel.HorizontalAlignment = 'right';
            app.alpha4SliderLabel.FontSize = 10;
            app.alpha4SliderLabel.Position = [200 185 35 22];
            app.alpha4SliderLabel.Text = 'alpha4';

            % Create alpha4Slider
            app.alpha4Slider = uislider(app.MotionPanel);
            app.alpha4Slider.Limits = [0 0.05];
            app.alpha4Slider.FontSize = 10;
            app.alpha4Slider.Position = [256 194 110 3];
            app.alpha4Slider.Value = 0.005;

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
            app.DeadReckButton.Value = true;

            % Create OdometryButton
            app.OdometryButton = uiradiobutton(app.ModelButtonGroup);
            app.OdometryButton.Text = 'Odometry';
            app.OdometryButton.Position = [11 12 75 22];

            % Create alpha5SliderLabel
            app.alpha5SliderLabel = uilabel(app.MotionPanel);
            app.alpha5SliderLabel.HorizontalAlignment = 'right';
            app.alpha5SliderLabel.FontSize = 10;
            app.alpha5SliderLabel.Position = [15 117 35 22];
            app.alpha5SliderLabel.Text = 'alpha5';

            % Create alpha5Slider
            app.alpha5Slider = uislider(app.MotionPanel);
            app.alpha5Slider.Limits = [0 0.05];
            app.alpha5Slider.FontSize = 10;
            app.alpha5Slider.Position = [71 126 110 3];
            app.alpha5Slider.Value = 0.005;

            % Create alpha6SliderLabel
            app.alpha6SliderLabel = uilabel(app.MotionPanel);
            app.alpha6SliderLabel.HorizontalAlignment = 'right';
            app.alpha6SliderLabel.FontSize = 10;
            app.alpha6SliderLabel.Position = [200 117 35 22];
            app.alpha6SliderLabel.Text = 'alpha6';

            % Create alpha6Slider
            app.alpha6Slider = uislider(app.MotionPanel);
            app.alpha6Slider.Limits = [0 0.05];
            app.alpha6Slider.FontSize = 10;
            app.alpha6Slider.Position = [256 126 110 3];
            app.alpha6Slider.Value = 0.005;

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

            % Create DistrubutionButtonGroup
            app.DistrubutionButtonGroup = uibuttongroup(app.MotionPanel);
            app.DistrubutionButtonGroup.Title = 'Distrubution';
            app.DistrubutionButtonGroup.Position = [23 22 186 51];

            % Create GaussianButton
            app.GaussianButton = uiradiobutton(app.DistrubutionButtonGroup);
            app.GaussianButton.Text = 'Gaussian';
            app.GaussianButton.Position = [11 5 73 22];
            app.GaussianButton.Value = true;

            % Create TriangleButton
            app.TriangleButton = uiradiobutton(app.DistrubutionButtonGroup);
            app.TriangleButton.Text = 'Triangle';
            app.TriangleButton.Position = [86 5 65 22];

            % Create ExternalDisturbanceCheckBox
            app.ExternalDisturbanceCheckBox = uicheckbox(app.MotionPanel);
            app.ExternalDisturbanceCheckBox.Text = 'External Disturbance';
            app.ExternalDisturbanceCheckBox.Position = [247 36 133 22];

            % Create LandmarkPanel
            app.LandmarkPanel = uipanel(app.UIFigure);
            app.LandmarkPanel.Title = 'Landmark';
            app.LandmarkPanel.Position = [573 274 215 156];

            % Create SigNumEditFieldLabel
            app.SigNumEditFieldLabel = uilabel(app.LandmarkPanel);
            app.SigNumEditFieldLabel.HorizontalAlignment = 'right';
            app.SigNumEditFieldLabel.Position = [21 107 58 22];
            app.SigNumEditFieldLabel.Text = 'Sig. Num.';

            % Create SigNumEditField
            app.SigNumEditField = uieditfield(app.LandmarkPanel, 'numeric');
            app.SigNumEditField.Position = [94 107 100 22];
            app.SigNumEditField.Value = 3;

            % Create SignatureDropDownLabel
            app.SignatureDropDownLabel = uilabel(app.LandmarkPanel);
            app.SignatureDropDownLabel.HorizontalAlignment = 'right';
            app.SignatureDropDownLabel.Position = [25 81 57 22];
            app.SignatureDropDownLabel.Text = 'Signature';

            % Create SignatureDropDown
            app.SignatureDropDown = uidropdown(app.LandmarkPanel);
            app.SignatureDropDown.Items = {'1', '2', '3'};
            app.SignatureDropDown.Position = [97 81 100 22];
            app.SignatureDropDown.Value = '1';

            % Create MxmEditFieldLabel
            app.MxmEditFieldLabel = uilabel(app.LandmarkPanel);
            app.MxmEditFieldLabel.HorizontalAlignment = 'right';
            app.MxmEditFieldLabel.Position = [15 47 43 22];
            app.MxmEditFieldLabel.Text = 'Mx (m)';

            % Create MxmEditField
            app.MxmEditField = uieditfield(app.LandmarkPanel, 'numeric');
            app.MxmEditField.Position = [73 47 22 22];

            % Create MymEditFieldLabel
            app.MymEditFieldLabel = uilabel(app.LandmarkPanel);
            app.MymEditFieldLabel.HorizontalAlignment = 'right';
            app.MymEditFieldLabel.Position = [114 47 43 22];
            app.MymEditFieldLabel.Text = 'My (m)';

            % Create MymEditField
            app.MymEditField = uieditfield(app.LandmarkPanel, 'numeric');
            app.MymEditField.Position = [172 47 22 22];

            % Create AcceptButton
            app.AcceptButton = uibutton(app.LandmarkPanel, 'push');
            app.AcceptButton.ButtonPushedFcn = createCallbackFcn(app, @AcceptButtonPushed, true);
            app.AcceptButton.Position = [15 11 83 22];
            app.AcceptButton.Text = 'Accept';

            % Create RandomGenButton
            app.RandomGenButton = uibutton(app.LandmarkPanel, 'push');
            app.RandomGenButton.ButtonPushedFcn = createCallbackFcn(app, @RandomGenButtonPushed, true);
            app.RandomGenButton.Position = [110 11 91 22];
            app.RandomGenButton.Text = 'Random Gen.';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Robot Space')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.PlotBoxAspectRatio = [1.52216748768473 1 1];
            app.UIAxes.XLim = [-6 6];
            app.UIAxes.YLim = [-6 6];
            app.UIAxes.Position = [22 334 533 380];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = final_ekf_particle_exported

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