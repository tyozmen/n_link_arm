classdef bball_1_dof_Env_apex_control < rl.env.MATLABEnvironment  
    %% Properties 
    properties
        
        g = 9.8;    % The constant acceleration in y. 
        r = 0.1;    % radius of the ball
        d = 0.05    % half thickness of the link
        m_s = 1;    % mass of link
        m_b = .15;  % mass of ball 
        e = .8;     % coefficient of restitution
        
        dt = .01; 

        N = 1000; % how many steps i n an episode %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        lim = 890; % max force. This is about 200lb-force which can be achieved by a $130 linear actuator
        ptch=[]
        init_qvals = [];
        Figure = [];
        q_vrs = [];
        full_vrs = [];
        % Initialize system state [y_s,y_b,dy_s,dy_b]'
        X = [];
        curStep = 0;
        t = [];
        states_arr = [];
        actions_arr = [];
        t_arr = [];
        animate = false;
        
        
        usePDControl = true; % wether to use the model based control or not
        % Variables needed for apex control
        t_ni = 0;       % time until next impact
        h_apx = 0;      % apex height;
        h_d_apx = 3-.1;    % desired apex;
        y_imp = 2-.05;      % pre-set impact height
        
        y_paddle_max = 2 +.2;
        y_paddle_min = 0;
        
            % Where the link trajectory is defined as A*sin(2*pi*fr*t_sin)
        A = 0.35;       % amplitude of the sinusoidal for the link
        fr = .5;        % frequency required for the desired impact velocity
        t_sin = 0       % time value for the sinusoidal trajectory
        
        t_m = [];       % time required to move the link before the ball hits
        t_ai = 0;       % time after impact
        t_idle = 100;   % how long the arm should wait idle before getting ready to hit the ball initially
        f_idle = 0;     % flag for idle state
        y_des = 2;      % desired y position for the link
        dy_des = 0;     % desired y velocity for the link
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false;       
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = bball_1_dof_Env_apex_control(usePDControl)
            arguments
                usePDControl = true;
            end
           
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([4 1]);
            ObservationInfo.Name = '1DoF system states';
            ObservationInfo.Description = sprintf('y_s, y_b, dy_s, dy_b');
            
            % Initialize Action settings   
            lim = 890;
            ActionInfo = rlNumericSpec([1 1], 'LowerLimit', -ones(1,1), 'UpperLimit', ones(1,1));
            ActionInfo.Name = 'link Action';
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            
            this.usePDControl = usePDControl;
            this.lim = lim;
            this.init_qvals = [0:5*this.dt:pi];
            
            
            
%             this.states_arr = [];
%             this.actions_arr = [];
%             this.t_arr = [];
            this.t = 0;
            
          
            % Initialize property values and pre-compute necessary values
            % this.ActionInfo.Elements = this.MaxForce*[-1 1];

        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            LoggedSignals = [];
            Action = max(-1, Action);
            Action = min(1, Action);
            Action = Action*this.lim;
            
            q = this.X;
            
            %  time track
            %  this.t = this.t + this.dt;
            this.t_ai = this.t_ai + this.dt;
            
            if this.t_ai > this.t_idle && this.f_idle == 0 % then stepped too far
                t_ai_prev = this.t_ai - this.dt; % previous time after impact 
                dt2idle = this.t_idle-(t_ai_prev);
                this.t_ai = t_ai_prev + dt2idle;
                this.dt = dt2idle;
                this.f_idle = 1; % turn idle flag to 1 
            else
                this.dt = 0.01;
            end
            
            if this.usePDControl
                % Feedback Lin. + PD control for now to test
                kp = 55;
                kd = 111.5;
                this.y_des = this.y_imp - this.A*sin(2*pi*this.fr*this.t_sin);
                if this.t_sin == 0
                    this.dy_des = this.y_des-q(1);
                else
                    this.dy_des = -this.A*cos(2*pi*this.fr*this.t_sin)*2*pi*this.fr;
                end

                u_fl = this.m_s*this.g;
                u = u_fl + kp*(this.y_des-q(1))+kd*(this.dy_des - q(3)); % to compare with bball_1_dof_main
                %force limits
                u = max(-this.lim, u);
                u = min(this.lim, u);
                Action = u;
            else
                u = Action;
            end
            
            dq = [this.X(3); this.X(4); u/this.m_s-this.g; -this.g];
            q = q + dq*this.dt;
           
            if q(1) >= this.y_paddle_max
                q(1) = this.y_paddle_max;
                q(3) = 0; dq(1) = 0;
            elseif q(1) < this.y_paddle_min
                q(1) = this.y_paddle_min;
                q(3) = 0; dq(1) = 0;
            end
            
            if (q(1)+this.d) > (q(2)-this.r) % then ball went through the link. now let's find where contact happens
                dt_temp = ((this.X(2)-this.r) - (this.X(1)+this.d))/(this.X(3)-this.X(4)); % dt to reach contact point
                
                if this.usePDControl
                    this.t_sin = this.t_sin-this.dt+dt_temp;
                    this.y_des = this.y_imp - this.A*sin(2*pi*this.fr*this.t_sin);
                    this.dy_des = -this.A*cos(2*pi*this.fr*this.t_sin)*2*pi*this.fr;
                    u_fl = this.m_s*this.g;
                    u = u_fl + kp*(this.y_des-q(1))+kd*(this.dy_des - q(3));
                    %force limits
                    u = max(-this.lim, u);
                    u = min(this.lim, u);
                    Action = u;
                else 
                    u = Action;
                end
                
                dq = [this.X(3); this.X(4); u/this.m_s-this.g; -this.g];
                
                
                q = this.X + dq * dt_temp; % find the states right at the impact

                if q(1) >= this.y_paddle_max
                    q(1) = this.y_paddle_max;
                    q(3) = 0; dq(1) = 0;
                elseif q(1) < this.y_paddle_min
                    q(1) = this.y_paddle_min;
                    q(3) = 0; dq(1) = 0;
                end

                
                % pre-impact velocities right when they touch
                dy_si = q(3);  % pre-impact link velocity
                dy_bi = q(4);  % pre-impact ball velocity
                
                
                % post-impact velocities

                Coeffs = [this.m_b this.m_s; 1 -1];
                dy_f = Coeffs\[this.m_b*dy_bi+this.m_s*dy_si; (dy_si-dy_bi)*this.e];
                dy_bf = dy_f(1);
                dy_sf = dy_f(2);
                
                dy_bi_nxt = -dy_bf; % post impact ball vel now is the next pre-impact ball vel
                
                this.t_ni = 2*dy_bf/this.g; 
                this.h_apx = q(2)+(dy_bf^2)/(2*this.g);
                
                dybf_d = sqrt(this.h_d_apx*2*this.g); % desired ball vel after impact

                dys_d = [1 -1; 1 this.e]\[this.m_b*(dy_bi_nxt-dybf_d)/this.m_s; dybf_d+this.e*dy_bi_nxt];
                dysi_d = dys_d(2);
                % the surface velocity at the point of impact is set to be
                % A*2*pi*f.
                this.fr = dysi_d/(this.A*2*pi); %required frequency of sinusoidal to hit the ball with the right velocity
                
                this.t_m = 1/(2*this.fr);
                this.t_idle = this.t_ni-this.t_m;
                this.t_ai = 0;
                this.f_idle = 0;
                this.t_sin = 0;
               
                
                q(3) = dy_sf;
                q(4) = dy_bf;
                
                
               
                this.t = this.t + dt_temp;
            else
                this.t = this.t + this.dt;
                %  this.t_ai = this.t_ai + this.dt;
                if this.t_ai > this.t_idle
                    this.t_sin = this.t_sin+this.dt;
                else
                    this.t_sin = 0;
                end
            end
            

            this.X = q;
            Observation = this.X;
            
            
            this.states_arr = [this.states_arr Observation];
            this.actions_arr = [this.actions_arr Action];
            this.t_arr = [this.t_arr this.t];
 
            Reward = -1e-2.*((this.h_d_apx + this.y_imp)-this.X(2)).^2;
           
            IsDone = this.curStep >= this.N; %|| term;
            this.curStep = this.curStep + 1;
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            
%             if this.animate
%                 notifyEnvUpdated(this);
%             end
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
%             this.X(1:this.n,1) = randsample(this.init_qvals,this.n);
%             this.X(this.n+1:end,1) = 0;
            this.X = [2-this.d; 7; 0; 0]; % to compare with bball_1_dof_main
            this.states_arr = [];
            this.actions_arr = [];
            this.t_arr = [];
            this.t = 0;
            this.t_ai = 0; 
            this.t_sin = 0;
            this.t_idle = 100;     
            this.f_idle = 0;
            
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
        % Discrete force 1 or 2
%         function force = getForce(this,action)
%             if ~ismember(action,this.ActionInfo.Elements)
%                 error('Action must be %g for going left and %g for going right.',-this.MaxForce,this.MaxForce);
%             end
%             force = action;           
%         end
%         % update the action info based on max force
%         function updateActionInfo(this)
%             this.ActionInfo.Elements = this.MaxForce*[-1 1];
%         end
%         
%         % Reward function
%         function Reward = getReward(this)
%             if ~this.IsDone
%                 Reward = this.RewardForNotFalling;
%             else
%                 Reward = this.PenaltyForFalling;
%             end          
%         end
%         
%         % (optional) Visualization method
%         function plot(this)
%             this.Figure = figure('Color','w','Renderer','zbuffer');
%             ha = gca(this.Figure);
%             ha.XLimMode = 'manual';
%             ha.YLimMode = 'manual';
%             ha.XLim = [-5 5];
%             ha.YLim = [-5 5];
% 
%             % Update the visualization
% %             envUpdatedCallback(this)
%         end
        
        function [obs_arr, u_arr, t_arr] = get_arrays(this)
            obs_arr = this.states_arr;
            u_arr = this.actions_arr;
            t_arr = this.t_arr;
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