classdef n_link_ball_Env < rl.env.MATLABEnvironment
    %fast environment for the n-link arm. Uses anonymous functions for M,C,Tg instead of symbolics    
    
    %% Properties 
    properties
        
        g = 9.8 % The constant acceleration in y. 
        M = [];
        C = [];
        Tg = [];
        B = [];
        n = 3; % number of links
        L = [];
        
        r = 0.1;    % radius of the ball
        d = 0.05;   % half thickness of the link
        e = .8;     % coeff of restitution
        m_b = .15;

        dt_min = 1e-6;
        
        dt = .01; 

        N = 2500; % how many steps i n an episode %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        lim = 35; % bound for torques
        ptch=[]
        init_qvals = [];
        Figure = [];
        q_vrs = [];
        full_vrs = [];
        
        % Initialize system state [x,dx,y,dy]'
        X = [];
        curStep = 0;
        t = [];
        states_arr = [];
        actions_arr = [];
        t_arr = [];
        animate = false;
        
        %q_des = [-pi/3; pi/3; pi/2];
        q_des = [-pi/6; 7*pi/12; pi/2]
        x_init = 1.0;
        x_max = 4.0;
        y_init = 4;
        usePDControl = true; % wether to use the model based control or not
        % Variables needed for apex control
        h_d_apx = 2-.1;    % desired apex;
        y_imp = 2-.05;      % pre-set impact height
        h_apx = 0;
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false;       
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = n_link_ball_Env(M,C,Tg,B,L,usePDControl)
            n = length(B); %number of links
           
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([2*n+4 1]);
            ObservationInfo.Name = 'n_link and ball system states';
            ObservationInfo.Description = sprintf('q1-q%d, x_b, y_b, dq1-dq%d, dx_b, dy_b',n,n);
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([n 1], 'LowerLimit', -ones(n,1), 'UpperLimit', ones(n,1));
            ActionInfo.Name = 'n_link Action';
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            this.init_qvals = [0:5*this.dt:pi];
            
            this.n = n;
            this.L = L;
            this.X = zeros(2*n+4,1);
            
            this.M = M;
            this.C = C;
            this.Tg = Tg;
            this.B = B;
            
            this.usePDControl = usePDControl;
            this.t = 0;
          
            % Initialize property values and pre-compute necessary values
            % this.ActionInfo.Elements = this.MaxForce*[-1 1];

        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            LoggedSignals = [];
            isDone = 0;
            Action = max(-1, Action);
            Action = min(1, Action);
            Action = Action*this.lim;
            
            % states
            q = this.X(1:this.n);
            q_b = this.X(this.n+1:this.n+2);
            
            dq = this.X(this.n+3:end-2);
            dq_b =this.X(end-1:end);
            
            
            Tg_tmp = this.Tg(q);
            M_tmp = this.M(q);
            C_tmp = this.C(this.X);
            
            if this.usePDControl
                % Feedback Lin. + PD control for now to test
                Kp = 15;
                Kd = 1;
                u_fl = double(C_tmp*this.X(this.n+3:end-2) + Tg_tmp*this.g);
                u = u_fl+Kp.*rad2deg(this.q_des-this.X(1:this.n)) + Kd*rad2deg(-this.X(this.n+3:end-2));
%                 this.y_des = this.y_imp - this.A*sin(2*pi*this.fr*this.t_sin);
%                 if this.t_sin == 0
%                     this.dy_des = this.y_des-q(1);
%                 else
%                     this.dy_des = -this.A*cos(2*pi*this.fr*this.t_sin)*2*pi*this.fr;
%                 end
% 
%                 u_fl = this.m_s*this.g;
%                 u = u_fl + kp*(this.y_des-q(1))+kd*(this.dy_des - q(3)); % to compare with bball_1_dof_main
%                 %force limits
%                 u = max(-this.lim, u);
%                 u = min(this.lim, u);
                Action = u;
            else
                u_fl = double(C_tmp*this.X(this.n+3:end-2) + Tg_tmp*this.g);
                Action = Action + u_fl;

            end
            
            d2q = double(-M_tmp\C_tmp*dq-M_tmp\Tg_tmp*this.g+M_tmp\this.B*Action);
            d2q_b = [0; -this.g];
            
            q = q + dq*this.dt;
            dq = dq + d2q*this.dt;
            
            q_b = q_b + dq_b*this.dt;
            dq_b = dq_b +d2q_b*this.dt;
            
            % need to detect collision
            Q = [q;q_b;dq;dq_b];

            [contact, upper ,lower, cornerHit] = contact_detect(this,Q);
            th1 = atan(cos(Q(this.n))/sin(Q(this.n)));
            th2 = atan(cos(this.X(this.n))/sin(this.X(this.n)));

            if contact
                
                if (abs(th1) > pi/4 || abs(th2) > pi/4)
                    isDone = 1;
                elseif cornerHit   % If the ball is going to hit the corner of the arm the episode is done
                    isDone = 1;
                else 
                   dt_temp = find_dt2contact(this,upper,lower,Q);
                    if dt_temp == -1
                        isDone = 1;
                    end
                end
            end
            if contact && ~isDone
                    if this.usePDControl
                        % Feedback Lin. + PD control for now to test
                        Kp = 55;
                        Kd = 111.5;
                        u_fl = double(C_tmp*this.X(this.n+3:end-2) + Tg_tmp*this.g);
                        u = u_fl+Kp.*rad2deg(this.q_des-this.X(1:this.n)) + Kd*rad2deg(-this.X(this.n+3:end-2));
    %                     this.y_des = this.y_imp - this.A*sin(2*pi*this.fr*this.t_sin);
    %                     if this.t_sin == 0
    %                         this.dy_des = this.y_des-q(1);
    %                     else
    %                         this.dy_des = -this.A*cos(2*pi*this.fr*this.t_sin)*2*pi*this.fr;
    %                     end
    % 
    %                     u_fl = this.m_s*this.g;
    %                     u = u_fl + kp*(this.y_des-q(1))+kd*(this.dy_des - q(3)); % to compare with bball_1_dof_main
    %                     %force limits
                        u = max(-this.lim, u);
                        u = min(this.lim, u);
                        Action = u;
                    else
                        u = Action;
                    end

                    d2q = double(-M_tmp\C_tmp*dq-M_tmp\Tg_tmp*this.g+M_tmp\this.B*Action);
                    dq = [this.X(this.n+3:end); d2q; 0; -this.g];
                    q = this.X + dq*dt_temp;    % pre-impact states 
                    
                    Q = impact(this,q);         % post-impact states
                    this.h_apx = Q(this.n+2) + Q(this.n+2+this.n+2)^2/(2*this.g);

                    this.t = this.t + dt_temp;
                  
           else
             this.t = this.t + this.dt;
           end
            
            this.X = Q;
            Observation = this.X;
            
            this.states_arr = [this.states_arr Observation];
            this.actions_arr = [this.actions_arr Action];
            this.t_arr = [this.t_arr this.t];
            
            ypen = -1e-1*((this.h_d_apx + this.y_imp)-this.h_apx).^2;
            xpen = -1e-2*((this.x_init)-this.X(this.n+1)).^2;
            alive = 1;
            Reward = alive + xpen + ypen;

            ball_x = this.X(this.n+1);
            if abs(ball_x) > this.x_max
                isDone = 1;
            end
           
            IsDone = isDone || this.curStep >= this.N; %|| term;
            this.curStep = this.curStep + 1;
       
            if this.X(this.n+2)-this.r <= 0 % ball is on the floor
                IsDone = 1;
            end
            
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
            this.X(1:this.n,1) = this.q_des; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% need to change this to random around initial condition
            this.X(this.n+1:this.n+2,1) = [this.x_init; this.y_init];
            this.X(this.n+3:end-2,1) = 0;
            this.X(end-1,1) = 0;
            this.X(end,1) = 0;
            this.states_arr = [];
            this.actions_arr = [];
            this.t_arr = [];
            this.t = 0;
            
            InitialObservation = this.X;
            this.states_arr = [this.states_arr InitialObservation];
            
            this.t_arr = [this.t_arr 0];
%             this.ptch = [];
            this.curStep = 0; 
            
            % notifyEnvUpdated(this);
        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods   
        
        
        % Helper methods to create the environment
        
        
        function [obs_arr, u_arr, t_arr] = get_arrays(this)
            obs_arr = this.states_arr;
            u_arr = this.actions_arr;
            t_arr = this.t_arr;
        end
        
       

        function [xb, yb, xr_u, yr_u, xr_l, yr_l, slope, xshift, yshift, b, th,xn,yn, ee] = closest_point(this, State)
             % Q is the next states the system will arive so check for
            % contact
            ee = n_link_fwdKin(this,State(1:this.n));
            xb = State(this.n+1);
            yb = State(this.n+2);

            xn = ee(1) - this.L(this.n)*sin(State(this.n));    % x coordinate of nth link
            yn = ee(2) - this.L(this.n)*cos(State(this.n));    % y coordinate of the nth link
            slope = cos(State(this.n))/sin(State(this.n));


            b = yn - slope*xn;

            th = atan2(cos(State(this.n)),sin(State(this.n))); % angle of the surface wrt x axis

            yshift = this.d*cos(th);
            xshift = this.d*sin(th);
            % equation for the surface of the arm
            % y = slope*x + b + yshift
            
            % closest point on the ball to the surface of the arm
            % upper
            xr_u = xb + sin(th)*this.r; 
            yr_u = yb - cos(th)*this.r;
            %lower
            xr_l = xb - sin(th)*this.r;
            yr_l = yb + cos(th)*this.r;

        end
           
        function [xr_u,yr_u, xr_l, yr_l, xbR,xnR,xeeR, eqn_upper, eqn_lower, canHappenU, canHappenL] = canHappen(this,Q)
           [xb, yb, xr_u, yr_u, xr_l, yr_l, slope, xshift, yshift, b, th, xn, yn, ee] = closest_point(this,Q);
            % y = slope*x+b  line equation of the arm
            % equations to check for upper and lower surfaces
            eqn_upper = @(x,y)(slope*(x+xshift)+ b + yshift - y)*-1;
            eqn_lower = @(x,y)(slope*(x-xshift)+ b - yshift - y)*-1;
              
            
                
            % project xn and ee to the surface of the arm to get the
            % surface end points
            
            % upper surface
            xnp_u = xn - this.d*sin(th);
            ynp_u = yn + this.d*cos(th);

            xeep_u = ee(1) - this.d*sin(th);
            yeep_u = ee(2) + this.d*cos(th);
            
            % lower surface
            xnp_l = xn + this.d*sin(th);
            ynp_l = yn - this.d*cos(th);

            xeep_l = ee(1) + this.d*sin(th);
            yeep_l = ee(2) - this.d*cos(th);

            % Change of basis matrix (to switch to the rotated coordinate frame)
            R = [cos(th) sin(th); -sin(th) cos(th)];
            
            


            % upper surface nth joint coordinates in the rotated coordinate frame
            np_u = R*[xnp_u; ynp_u];
            xnpR_u = np_u(1);
            ynpR_u = np_u(2);

            % lower surface nth joint coordinates in the rotated coordinate frame
            np_l = R*[xnp_l; ynp_l];
            xnpR_l = np_l(1);
            ynpR_l = np_l(2);
            
            % upper surface ee coordinates in the rotated coordinate frame
            ee_u = R*[xeep_u ; yeep_u];
            xeepR_u = ee_u(1);
            yeepR_u = ee_u(2);

            % lower surface ee coordinates in the rotated coordinate frame
            ee_l = R*[xeep_l; yeep_l];
            xeepR_l = ee_l(1);
            yeepR_l = ee_l(2);

            
            r_u = R*[xr_u; yr_u];
            xrR_u = r_u(1);
            yrR_u = r_u(2);


            r_l = R*[xr_l; yr_l];
            xrR_l = r_l(1);
            yrR_l = r_l(2);

            % ball coordinates in the rotated coordinate frame
            bR = R*[xb; yb];
            xbR = bR(1);
            ybR = bR(2);

            nR = R*[xn; yn];
            xnR = nR(1);

            eeR = R*ee(1:2,1);
            xeeR = eeR(1);
            % can the contact happen? is the ball within the x values of the last link
            % 
            % upper side
            if xeepR_u > xnpR_u && xrR_u >= xnpR_u && xrR_u <= xeepR_u
                canHappenU = 1;
            elseif xeepR_u < xnpR_u && xrR_u <= xnpR_u && xrR_u >= xeepR_u
                canHappenU = 2;
            else
                canHappenU = 0;
            end
            % lower side
            if xeepR_l > xnpR_l && xrR_l >= xnpR_l && xrR_l <= xeepR_l
                canHappenL = 3;
            elseif xeepR_l < xnpR_l && xrR_l <= xnpR_l && xrR_l >= xeepR_l
                canHappenL = 4;
            else
                canHappenL = 0;
            end 
        end
        
        function [contact,upper,lower,cornerHit] = contact_detect(this,Q)

            [xr_u, yr_u, xr_l, yr_l, xbR, xnR, xeeR, eqn_upper, eqn_lower, canHappenU, canHappenL] = canHappen(this,Q);
            
            % values from the previous time step
            [prv_xr_u, prv_yr_u, prv_xr_l, prv_yr_l, prv_xbR, prv_xnR, prv_xeeR, prv_eqn_upper, prv_eqn_lower, prv_canHappenU, prv_canHappenL] = canHappen(this,this.X);

            % is the closest point on the ball to the surface of the arm on the arm?
%             eqn = slope*xr + b + yshift - yr;
%             eqn =-eqn;
%             
%             [xrp, yrp, slopep, yshiftp, bp, thp, xnp, ynp, eep]  = closest_point(this,this.X);
%             eqnp = slopep*xrp + bp + yshiftp - yrp;
%             eqnp =-eqnp;


%             if eqnp > 0 && eqn < 0 && canHappen % if eqn went below zero and contact could have happened then integration went too far we need to take a step back
%                 contact = 1;       
%             elseif eqnp < 0 && eqn > 0 && canHappen
%                 contact = 1;
%             else
%                 contact = 0;
%             end
            
            
            % if eqn went below zero and contact could have happened then integration went too far we need to take a step back    
            if (eqn_upper(xr_u,yr_u) < 0 && eqn_lower(xr_u,yr_u) > 0 && canHappenU) ...
                    || (eqn_upper(xr_u,yr_u) > 0 && eqn_lower(xr_u,yr_u) < 0 && canHappenU)...
                    || (eqn_upper(xr_u,yr_u) < 0 && prv_eqn_upper(prv_xr_u,prv_yr_u) > 0 && prv_eqn_upper(prv_xr_l,prv_yr_l) > 0 && canHappenU) ...
                    || (eqn_upper(xr_u,yr_u) > 0 && prv_eqn_upper(prv_xr_u,prv_yr_u) < 0 && prv_eqn_upper(prv_xr_l,prv_yr_l) < 0 && canHappenU)
                
                contact = 1; lower = 0; upper = 1; cornerHit = 0;     

            elseif (eqn_upper(xr_l,yr_l) < 0 && eqn_lower(xr_l,yr_l) > 0 && canHappenL)...
                    || (eqn_upper(xr_l,yr_l) > 0 && eqn_lower(xr_l,yr_l) < 0 && canHappenL)...
                    || (eqn_lower(xr_l,yr_l) > 0 && prv_eqn_lower(prv_xr_l,prv_yr_l) < 0 && prv_eqn_upper(prv_xr_l,prv_yr_l) < 0 && canHappenL)...
                    || (eqn_lower(xr_l,yr_l) < 0 && prv_eqn_lower(prv_xr_l,prv_yr_l) > 0 && prv_eqn_upper(prv_xr_l,prv_yr_l) > 0 && canHappenL)
                    
                contact = 2; lower = 1; upper = 0; cornerHit = 0;
%                     (((eqn_upper(xr_u,yr_u) <= 0 && eqn_lower(xr_u,yr_u) > 0)||(eqn_upper(xr_l,yr_l) < 0 && eqn_lower(xr_l,yr_l) >= 0)) || ...
%                     ((eqn_upper(xr_u,yr_u) < 0 && eqn_lower(xr_u,yr_u) < 0)&&(eqn_upper(xr_l,yr_l) > 0 && eqn_lower(xr_l,yr_l) > 0))) &&...
%                     (abs(xbR-xnR) <= this.r || abs(xbR-xeeR) <= this.r)
            elseif (canHappenL || canHappenU) == 0 && ...
                     (  (eqn_upper(xr_u,yr_u) <= 0 && eqn_lower(xr_u,yr_u) > 0) ...
                    ||  (eqn_upper(xr_l,yr_l) < 0 && eqn_lower(xr_l,yr_l) >= 0)  ...
                    ||  (eqn_upper(xr_u,yr_u) >= 0 && eqn_lower(xr_u,yr_u) < 0) ...
                    ||  (eqn_upper(xr_l,yr_l) > 0 && eqn_lower(xr_l,yr_l) <= 0) ...
                    ||  ( (eqn_upper(xr_u,yr_u) < 0 && eqn_lower(xr_u,yr_u) < 0) && (eqn_upper(xr_l,yr_l) > 0 && eqn_lower(xr_l,yr_l) > 0))) ...
                    &&  (abs(xbR-xnR) <= this.r || abs(xbR-xeeR) <= this.r)

%             elseif ((canHappenL || canHappenU) == 0) && (eqn_upper(xb,yb) < 0 && eqn_lower(xb,yb) > 0) && (abs(xbR-xnR) <= this.r || abs(xbR-xeeR) <= this.r)
                contact = 0; lower = 0; upper = 0; cornerHit = 1;
                
            else
                contact = 0; lower = 0; upper = 0; cornerHit = 0;
            end
        end
        
        function [dt_temp] = find_dt2contact(this,upper,lower,Q)
            
            % to solve for what the necessary timestep needs to be we need
            % to write everything interms of dt and solve for the contact
            % condition
            
            q = this.X(1:this.n);
            q_b = this.X(this.n+1:this.n+2);

            dq = this.X(this.n+3:end-2);
            dq_b = this.X(end-1:end);
            
            q_nxt = @(dt)q + dq*dt;            % next joint angles
            xb_nxt = @(dt)q_b(1) + dq_b(1)*dt; % next x_b
            yb_nxt = @(dt)q_b(2) + dq_b(2)*dt; % next y_b

%             Q_nxt = @(dt)[q_nxt(dt);q_b_nxt(dt)];
            
            
            x_ee = @(dt)this.L'*sin(q_nxt(dt)); % next x_ee (basically forward kinematics)
            y_ee = @(dt)this.L'*cos(q_nxt(dt)); % next y_ee
            
            q_n = @(dt)q(this.n)+dq(this.n)*dt; % next angle of the nth link
            
            
            xn = @(dt) x_ee(dt) - this.L(this.n)*sin(q_n(dt));    % x coordinate of nth link
            yn = @(dt) y_ee(dt) - this.L(this.n)*cos(q_n(dt));    % y coordinate of the nth link
            slope =@(dt)cos(q_n(dt))/sin(q_n(dt));

            % y = slope*x+b  line equation of the arm
            b = @(dt) yn(dt) - slope(dt)*xn(dt);
            
%             th = @(dt) atan(slope(dt)); % angle of the surface wrt x axis
            th = @(dt)atan2(cos(q_n(dt)),sin(q_n(dt)));

            yshift = @(dt) this.d*cos(th(dt));
%             yshift = this.d*cos(th);
            xshift = @(dt) this.d*sin(th(dt));
            % equation for the surface of the arm
            % y = slope*x + b + yshift
            
            % closest point on the ball to the surface of the arm
            if upper
                xr = @(dt) xb_nxt(dt) + sin(th(dt))*this.r;
                yr = @(dt) yb_nxt(dt) - cos(th(dt))*this.r;
                eqn = @(dt)(slope(dt)*(xr(dt)+xshift(dt)) + b(dt) + yshift(dt) - yr(dt))*-1; % root of this eqn is the dt value we need
            elseif lower
                xr = @(dt) xb_nxt(dt) - sin(th(dt))*this.r;
                yr = @(dt) yb_nxt(dt) + cos(th(dt))*this.r;
                eqn = @(dt)(slope(dt)*(xr(dt)-xshift(dt)) + b(dt) - yshift(dt) - yr(dt))*-1; % root of this eqn is the dt value we need
            end
            
%             % project xn and ee to the surface of the arm
%             xnp = @(dt) xn(dt) - this.d*sin(th(dt));
%             ynp = @(dt) yn(dt) + this.d*cos(th(dt));
%             
%             xeep = @(dt) x_ee(dt) - this.d*sin(th(dt)); 
%             yeep = @(dt) y_ee(dt) + this.d*cos(th(dt));
%             
%             % is the closest point on the ball to the surface of the arm on the arm?
% 
%             xynp2xyeep = @(dt) sqrt((xnp(dt)-xeep(dt))^2+(ynp(dt)-yeep(dt))^2);   % distance from xynp to xyeep
% 
%             xyr2xynp = @(dt) sqrt((xnp(dt)-xr(dt))^2+(ynp(dt)-yr(dt))^2);         % distance from xyr to xynp
%             xyr2xyeep = @(dt) sqrt((xr(dt)-xeep(dt))^2+(yr(dt)-yeep(dt))^2);      % distance from xyr to xyeep
% 
%             % if the closest point on the ball is on the surface of the arm then
%             % xynp2xyeep =  xyr2xynp + xyr2xyeep (if point C is on line segment AB then AB = AC + CB)
%             eqn = @(dt) xyr2xyeep(dt) + xyr2xynp(dt) - xynp2xyeep(dt);
            
%             eqn = @(dt)slope(dt)*xr(dt) + b(dt) + yshift(dt) - yr(dt); % root of this eqn is the dt value we need

            try
                dt_temp = fzero(eqn,[0 this.dt]);
            catch
                %eqn(0)
                %eqn(this.dt)
                dt_temp = -1;
            end
            

            %dt_temp = max(this.dt_min, fzero(eqn,[0 this.dt]))
            
        end
        
        function [q_post] = impact(this,q)
            % take pre-impact states and and spit post-impact ones
            
            [~,~,l_imp,th] = n_link_impact_point(this,q);    % get the impact point and the distance it happens from the nth link, th is the surface angle wrt x-axis
            J_imp = nlink_Jacobian(this,q,[this.L(1:this.n-1,1); l_imp]);      % get the Jacobian assuming the impact point is the end effector
            
            dz_imp_i = J_imp*q(this.n+3:end-2); % impact point velocities right before impact
            
            dx_si = dz_imp_i(1);  % x velocity of the surface right before impact
            dy_si = dz_imp_i(2);  % y velocity of the surface right before impact
            
            dx_bi = q(end-1);  % x velocity of the ball right before impact
            dy_bi = q(end);    % y velocity of the ball right before impact
            
            % need to rotate the coordinate system so our basis vectors are
            % perpendicular and parallel to the surface
            R = [cos(th) sin(th); -sin(th) cos(th)];
            Vr_bi = R*[dx_bi; dy_bi]; 
            Vr_si = R*[dx_si; dy_si];
            
            Vr_bf = zeros(2,1); % post impact ball velocities in the rotated coordinate system
            M_imp = this.M(q(1:this.n)); 
            
            F_mtrx = [0 0; this.m_b -this.m_b];
            K = R*J_imp(1:2,:)*(M_imp\J_imp(1:2,:)'*(R'*F_mtrx))+[0 0;0 -1];
            Vr_bf = K\([Vr_si(1); this.e*(Vr_bi(2)-Vr_si(2))] - R*J_imp(1:2,:)*q(this.n+3:end-2));
            Vr_bf(1) = Vr_bi(1);
            
            Ff = 0; % No friction between surface and the ball
            Fr = F_mtrx*[Vr_bi(1); Vr_bf(2)];
            
            % The impulse forces acting on the arm during impact are -N and -Ff.
            % We need to rotate the coordinate system back so we can get Fx_ee and
            % Fy_ee and the final ball velocities
            % rotating back to normal coordinate frame
            V_bf = R'*Vr_bf; 
            F = R'*Fr;
            
            % using the Eqn 13 from "Impact  Configurations  and  Measures for Kinematically  Redundant  and Multiple  Armed Robot Systems", Ian D. Walker
            delta_dq = M_imp\J_imp(1:2,:)'*F;
            
            q_post = [q(1:this.n+2,1); q(this.n+3:end-2,1)+delta_dq; V_bf(1); V_bf(2)];
        end
        
        function [x_imp,y_imp,l_imp,th] = n_link_impact_point(this,Q)
            % get the impact point at the moment of collision
            ee = n_link_fwdKin(this,Q(1:this.n));
            xb = Q(this.n+1);
            yb = Q(this.n+2);



            xn = ee(1) - this.L(this.n)*sin(Q(this.n));    % x coordinate of nth link
            yn = ee(2) - this.L(this.n)*cos(Q(this.n));    % y coordinate of the nth link
            slope = cos(Q(this.n))/sin(Q(this.n));

            % y = slope*x+b  line equation of the arm

            b = yn - slope*xn;

%             th = atan(slope); % angle of the surface wrt x axis
            th = atan2(cos(Q(this.n)),sin(Q(this.n)));
%             yshift = this.nd/cos(th);

            % equation for the surface of the arm
            % y = slope*x + b + yshift

            % closest point on the ball to the surface of the arm
            xr = xb + sin(th)*this.r;
            yr = yb - cos(th)*this.r;

%             eqn = slope*xr + b + yshift - yr;

            % project xr onto the center line of the arm

            x_imp = xr + this.d*sin(th);
            y_imp = yr - this.d*cos(th);
            l_imp = sqrt((xn - x_imp)^2+(yn-y_imp)^2);

        end
        
        function [ee] = n_link_fwdKin(this,Q)
            %where Q are the states and L is the vector of arm lengths
%             Q = this.X;
            q = Q(1:this.n,1);
            x_ee  = this.L'*sin(q);
            y_ee = this.L'*cos(q);
            th_ee = q(end);
            ee = [x_ee; y_ee; th_ee];
        end
        
        function [J] = nlink_Jacobian(this,Q,L)
            
            q = Q(1:this.n,1);
            J = zeros(3,this.n);

            for i = 1:this.n
                J(1,i) = L(i)*cos(q(i));
                J(2,i) = -L(i)*sin(q(i));
            end
            J(3,this.n) = 1;

        end

    end
    

    
%     methods (Access = protected)
%         % (optional) update visualization everytime the environment is updated 
%         % (notifyEnvUpdated is called)
%         
%         function envUpdatedCallback(this)
%             if ~isempty(this.Figure) && isvalid(this.Figure)
%             q = this.X(1:this.n,1);
%             L = ones(this.n,1);
%             p = this.ptch;
%             
%             if ~isempty(p)
%                 for j=1:this.n
%                     set(p(j), 'Visible', 'off') 
%                 end
%             end
%             for i = 1:this.n
%             
% 
%                 th = .06; % half-THickness of arm
% 
%                 avals = pi*[0:.05:1];
%                 x(i,:) = [0 L(i) L(i)+th*cos(avals-pi/2) L(i) 0 th*cos(avals+pi/2)];
%                 y(i,:) = [-th -th th*sin(avals-pi/2) th th th*sin(avals+pi/2)];
%                 r(i,:) = (x(i,:).^2 + y(i,:).^2).^.5;
%                 a(i,:) = atan2(y(i,:),x(i,:));
%                 if i ==1
%                     xdraw(:,i) = r(i,:).*sin(a(i,:)+q(i));  % x pts to plot, for Link i
%                     ydraw(:,i) = r(i,:).*cos(a(i,:)+q(i));  % y pts to plot, for Link i
%                     xend(i) = L(i)*sin(q(i));  % "elbow" at end of Link i, x
%                     yend(i) = L(i)*cos(q(i));  % "elbow" at end of Link i, x
%                 else
%                     xdraw(:,i) = xend(:,i-1) + r(i,:).*sin(a(i,:)+q(i));  % x pts to plot, for Link i
%                     ydraw(:,i) = yend(:,i-1) + r(i,:).*cos(a(i,:)+q(i));  % y pts to plot, for Link i
%                     xend(i) = xend(i-1) + L(i)*sin(q(i));  % "elbow" at end of Link i, x
%                     yend(i) = yend(i-1) + L(i)*cos(q(i));
%                 end
%                 
%             
%                 this.Figure; 
%                 p(i) = patch(xdraw(:,i),ydraw(:,i),'b','FaceAlpha',.3); hold on %pole i
%             end
%             this.ptch = p;
%             drawnow();
%             end
%         end
%     end
end
