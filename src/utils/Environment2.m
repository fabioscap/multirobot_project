classdef Environment2 < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dim
        dim_x

        N % the number of agents
        p_t % the pose of the target (unknown to agents)
        x % the unknown vector x (see references)
        p_hat % the current estimate of the target
        x_bnd = [-20; 20];
        y_bnd = [-20; 20];
        z_bnd = [-10; 10];
        
        % agents
        n_agents
        agents = []
        trajectories
        order = 4;
        positions
        graph


        tau % ARTVA sampling time
        last_sample = 0
        m=10   % ARTVA amplitude
        
        % coefficients for the approximation of the output
        ab = eq8(200); 

        % TODO validate these parameters
        % initializer for the S matrix in RLS
        S ;
        % forgetting factor
        beta = 0.8;


        % simulation parameters
        t=0
        
        % trajectory interval (you get this from solving opt problem)
        interval

        % replanning interval
        t_bar = 10;
        last_replan = 0


    end
    
    methods
        function obj = Environment2(dim, N, p_t, tau)
            %ENVIRONMENT Constructor

            obj.dim = dim;

            obj.n_agents = N;
            obj.p_t = p_t;

            obj.trajectories = zeros(dim, obj.order+1, obj.n_agents);
            obj.positions    = zeros(dim , 1, obj.n_agents);
            for i=1:obj.n_agents
                % TODO define other initialization methods for the agent
                % (maybe equally spaced around a point)
                if obj.dim == 2
                random_position = [obj.x_bnd(1) + rand()*(obj.x_bnd(2) - obj.x_bnd(1)); 
                                   obj.y_bnd(1) + rand()*(obj.y_bnd(2) - obj.y_bnd(1))];  
                elseif obj.dim==3
                random_position = [obj.x_bnd(1) + rand()*(obj.x_bnd(2) - obj.x_bnd(1)); 
                                   obj.y_bnd(1) + rand()*(obj.y_bnd(2) - obj.y_bnd(1));
                                   obj.z_bnd(1) + rand()*(obj.z_bnd(2) - obj.z_bnd(1))];     
                end
                obj.positions(:,1,i) = random_position;
                ai = Agent(random_position);
                if i == 1
                    obj.agents = [ai];
                else
                    obj.agents(i) = ai;
                end

            end

            obj.tau = tau;

            if dim == 2
                obj.dim_x = 6;
                obj.x = [obj.m 0 obj.m -1 1 0]';
            elseif dim == 3
                obj.dim_x = 10;
                obj.x = [obj.m 0 0 obj.m 0 obj.m 0 0 0 0]';
            end
            obj.S =0.1* eye(obj.dim_x, obj.dim_x);

            %This time p_hat is a vector of estimates, with a different
            %value for each agent.
            obj.p_hat = zeros(dim, N);

            %Here we add the construction of the graph. The connections
            %between the agents is simple: each agent i is connected only
            %with agents i-1 and i+1.
            graph = zeros(N,N);
            for s = 1:N
                if s==1
                    graph(s,N) = 1;
                    graph(s,s) = 1;
                    graph(s,s+1) = 1;
                elseif s==N
                    graph(s,s-1) = 1;
                    graph(s,s) = 1;
                    graph(s,1) = 1;
                else
                    graph(s,s-1) = 1;
                    graph(s,s) = 1;
                    graph(s,s+1) = 1;
                end
            end
            obj.graph = graph;
           end
        
        % RLS recursive step (eq. 5)
        
        function obj = RLSStep(obj)
            % we update the estimate of the unknown vector x
            % this has to be called whenever we receive 
            % a new ARTVA sample
            Y = zeros(obj.n_agents,1);
            H = zeros(obj.dim_x, obj.n_agents);
            for i=1:obj.n_agents
                p = obj.positions(:,1,i);

                % collect the individual measurements for each agent
                % TODO set noise to true
                sig_norm = getARTVAsig(p, obj.p_t, eye(3), eye(3), false, obj.m);
                % construct the output
                Y(i) = ( (obj.m/(sig_norm*4*pi))^(2/3) )* (obj.ab(1)*obj.ab(2))^2;
                % build phi vector
                H(:,i) = buildPhi(p);
            end

            K = obj.S * H /( obj.beta*eye(obj.n_agents, obj.n_agents) + H'*obj.S*H) ;
            obj.x = obj.x + K *(Y - H'*obj.x);
            obj.S = (1/obj.beta) * (obj.S - K*H'*obj.S);

            obj.p_hat = extractTarget(obj.x, obj.ab(1), obj.ab(2));

            obj.p_hat
        end



        % RLS recursive step modified for consensus protocol. In this
        % version, the estimate of the target's position is comouted for
        % each agent, based on only its neighbors
        function obj = RLSStep_Consensus(obj,agent_idx)
            % we update the estimate of the unknown vector x
            % this has to be called whenever we receive 
            % a new ARTVA sample
            Y = zeros(obj.n_agents,1);
            H = zeros(obj.dim_x, obj.n_agents);
            for i=1:obj.n_agents
                if obj.graph(agent_idx,i)==0
                    continue
                end
                p = obj.positions(:,1,i);

                % collect the individual measurements for each agent
                % TODO set noise to true
                sig_norm = getARTVAsig(p, obj.p_t, eye(3), eye(3), false, obj.m);
                % construct the output.Here we make a change w.r.t the
                % previous function by zeroing the entries of Y which are 
                % not part of the neighbourhood
                Y(i) = ( (obj.m/(sig_norm*4*pi))^(2/3) )* (obj.ab(1)*obj.ab(2))^2;
                % build phi vector
                H(:,i) = buildPhi(p);
            end

            K = obj.S * H /( obj.beta*eye(obj.n_agents, obj.n_agents) + H'*obj.S*H) ;
            obj.x = obj.x + K *(Y - H'*obj.x);
            obj.S = (1/obj.beta) * (obj.S - K*H'*obj.S);

            target_estimate = extractTarget(obj.x, obj.ab(1), obj.ab(2));

            obj.p_hat(:,agent_idx) = target_estimate;
        end

        % trajectory planning
        function obj = planTrajectories(obj)
            [obj.interval, obj.trajectories] = planningProblem(obj);
            for a=1:obj.n_agents
                obj.agents(a).updateReference(obj.trajectories(:,:,a), obj.interval);
            end
        end


        function obj = sim(obj)
            X = [];
            T = [];
            obj.RLSStep();
            obj.planTrajectories();


            % set up the events
            options = odeset("Events", @(t, y) obj.eventsFcn(t, y));

            % build the initial condition vector
            sz = 2*obj.dim;
            start_ = 1;
            for a=1:obj.n_agents
                agent = obj.agents(a);
                %
                x0(start_:start_+sz-1) = agent.x;
                %
                start_ = start_ + sz;
            end
            
            exit = false;
            while ~exit
                disp("Integrating from "+ obj.t + " to " +obj.interval(end));
                [t, x, te, xe, ie] = ode45(@(t,x) obj.f(t,x), [obj.t, obj.interval(end)], x0, options);
                X = [X; x];
                T = [T; t];
               
                disp("stopped at " + te);
                % replace x0 and obj.t accordingly to restart the integration
                % from where it stopped

                if isempty(te)
                    % no events...
                    disp("No events..")
                    obj.t = t(end);
                    x0 = x(end,:);

                else % event
                    % we set obj.position so we apply the updates
                    % at the correct position
                    sz = 2*obj.dim;
                    start_ = 1;
                    for a=1:obj.n_agents
                        %
                        obj.positions(:,:,a) = x(end,start_:start_+1);
                        %
                        start_ = start_ + sz;
                    end
                    if ie == 1
                        x0 = xe;
                        obj.t = te;
                        % do RLS...
                        obj.RLSStep();
                        %
                        obj.last_sample = te;
                    elseif ie == 2
                        x0 = xe;
                        t = te;
                        % do replanning...
                        obj.planTrajectories();
                        %
                        obj.last_replan = te;
                    else
                        % this should not happen
                        error("what?")
                    end
                end
                exit = t(end) > 100;
            end
            
            % TODO move this away
            % plot the positions of the agents in time
            f = figure();
            if obj.dim == 2
                scatter(obj.p_t(1), obj.p_t(2), "cyan","Marker", "o", 'LineWidth', 2); hold on
            else
                error("cannot plot 3d yet")
            end
            sz = 2*obj.dim;
            start_ = 1;
            for a=1:obj.n_agents
                agent = obj.agents(a);
                %
                if obj.dim == 2
                    plot(X(:,start_), X(:,start_+1),"b"); hold on;
                else
                    error("cannot plot 3d yet")
                end
                pause()
                %
                start_ = start_ + sz;
            end
        end

        % this function is the dynamic equations of
        % all the agents
        function xdot = f(obj, t, x)
            xdot = zeros(obj.n_agents*2*obj.dim + 0, 1);
            sz = 2*obj.dim;
            start_ = 1;

            for a=1:obj.n_agents
                agent = obj.agents(a);
                %
                xdot(start_:start_+sz-1) = agent.cl_dyn(t, x(start_:start_+sz-1));
                %
                start_ = start_ + sz;
            end
            % this is where you would add consensus on
            % the individual agent estimates
            % by allocating space on xdot (now there is a "+0")
            % it should look something like 
            % x_dot(start_:end) = -L*x(start_:end)
            %
            %
            %
            %
         
        end
        
        % this function servers the purpose of telling ode45 
        % that an event has occurred
        % we are interested in two kinds of event:
        % 1) new ARTVA sample (periodic)
        % 2) replanning conditions
        %
        % From documentation:
        % "If a terminal event occurs during the first step of the
        % integration, then the solver registers the event as nonterminal and continues integrating."
        function [value, isterminal, direction] = eventsFcn(obj, t, y)
            
            % new ARTVA signal?
            value(1) = t - obj.last_sample > obj.tau;
            % we want to stop the integration when this happens
            isterminal = 1;

            % TODO write better replanning conditions (like in paper2)
            value(2) = (t - obj.last_replan > obj.t_bar) || ...
                       t >= obj.interval(end);
            % we want to stop the integration when this happens
            isterminal(2) = 1;

            % direction does not matter in our case
            direction(1) = 0;
            direction(2) = 0;
        end
    end
end

