classdef bball_1_dof_Env < rl.env.MATLABEnvironment
    %fast environment for the n-link arm. Uses anonymous functions for M,C,Tg instead of symbolics    
    
    %% Properties 
    properties
        
        g = 9.8;    % The constant acceleration in y. 
        r = 0.1;    % radius of the ball
        d = 0.05    % half thickness of the link
        m_s = 1;    % mass of link
        m_b = .25;  % mass of ball 
        e = .8;     % coefficient of restitution
        
        dt = .01; 

        N = 1000; % how many steps i n an episode %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        lim = 1000; % bound for torques
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
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false;       
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = bball_1_dof_Env()
            
           
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([4 1]);
            ObservationInfo.Name = '1DoF system states';
            ObservationInfo.Description = sprintf('y_s, y_b, dy_s, dy_b');
            
            % Initialize Action settings   
            lim = 100;
            ActionInfo = rlNumericSpec([1 1], 'LowerLimit', -ones(1,1), 'UpperLimit', ones(1,1));
            ActionInfo.Name = 'link Action';
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            this.lim = lim;
            this.init_qvals = [0:5*this.dt:pi];
            
            
            
            this.states_arr = [];
            this.actions_arr = [];
            this.t_arr = [];
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
            u = Action;            
            
            q = this.X;
            
            % Feedback Lin. + PD control for now to test
            kp = 65;
            kd = 4.5;
            u = this.m_s*this.g+kp*(2-q(1))+kd*(0 - q(3)); % to compare with bball_1_dof_main
            
            dq = [this.X(3); this.X(4); u/this.m_s-this.g; -this.g];
            
            q = q + dq*this.dt;
            
            if (q(1)+this.d) > (q(2)-this.r) % then ball went through the link. now let's find where contact happens
                dt_temp = ((this.X(2)-this.r) - (this.X(1)+this.d))/(this.X(3)-this.X(4)); % dt to reach contact point
                
                q = this.X + dq * dt_temp;
                
                % pre-impact velocities right when they touch
                dy_si = q(3);  % pre-impact link velocity
                dy_bi = q(4);  % pre-impact ball velocity
                
                % post-impact velocities
                dy_bf = -this.e*(dy_bi-dy_si)+dy_si; 
                dy_sf = (1+this.e)*this.m_b*(dy_bi-dy_si)/this.m_s + dy_si;
                
                q(3) = dy_sf;
                q(4) = dy_bf;
                
                this.X = q;
                this.t = this.t + dt_temp;
            else
                this.X = q;    
                this.t = this.t + this.dt;
            end
 
            
            
            Observation = this.X;
            
            
            this.states_arr = [this.states_arr Observation];
            this.actions_arr = [this.actions_arr Action];
            this.t_arr = [this.t_arr this.t];
            
            
            % 0 for now will be changed to reflect a desired position
            Reward = 0; 
            % Reward = -1e-2.*sum((0-this.X).^2);
           
            IsDone = this.curStep >= this.N; %|| term;
            this.curStep = this.curStep + 1;
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            
            if this.animate
                notifyEnvUpdated(this);
            end
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
%             this.X(1:this.n,1) = randsample(this.init_qvals,this.n);
%             this.X(this.n+1:end,1) = 0;
            this.X = [0; 3; 0; 0]; % to compare with bball_1_dof_main
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