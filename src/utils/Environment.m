classdef Environment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dim
        
        N % the number of agents
        p_t % the pose of the target (unknown to agents)
        
        x_bnd = [-20; 20];
        y_bnd = [-20; 20];
        z_bnd = [-10; 10];
        
        % agents
        n_agents
        agents = []
        dyn_size
        trajectories
        positions

        % ARTVA signal amplitude
        m = 10

        tau % ARTVA sampling time
        last_sample = 0

        % instance that holds the target estimate
        estimate
        estimate_history = []
        
        % simulation parameters
        t=0
        
        dim_x

        % trajectory interval (you get this from solving opt problem)
        interval

        % replanning interval
        t_bar = 10;
        last_replan = 0

        % centralized agent does not need this
        extr_dim = 0


        ab = eq8(200);
    end
    
    methods
        function obj = Environment(dim, N, p_t, tau)
            %ENVIRONMENT Constructor

            obj.dim = dim;
            if dim == 2
                obj.dim_x = 6;
            elseif dim == 3
                obj.dim_x = 10;
            end
            obj.n_agents = N;
            obj.p_t = p_t;

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
                ai = AgentDec(random_position, obj.m, tau);
                if i == 1
                    obj.agents = [ai];
                    obj.dyn_size = ai.dyn_size;
                else
                    obj.agents(i) = ai;
                end

            end

            obj.tau = tau;

            obj.estimate = Estimate(dim, obj.m);

        end
        
        % RLS recursive step (eq. 5)
        function obj = updateEstimate(obj)
            sig_norms = zeros(obj.n_agents,1);
            for i=1:obj.n_agents
                p = obj.positions(:,1,i);
                sig_norms(i) = getARTVAsig(p, obj.p_t, eye(3), eye(3), true, obj.m);
            end
            obj.estimate.RLSStep(obj.positions, sig_norms);

            obj.estimate_history = [obj.estimate_history; [obj.t obj.estimate.value']];
        end

        % trajectory planning
        function obj = planTrajectories(obj)
            [obj.interval, obj.trajectories] = planningProblem(obj.positions, ...
                                                               obj.t, ...
                                                               obj.estimate.value, ...
                                                               obj.tau);
            for a=1:obj.n_agents
                obj.agents(a).updateReference(obj.trajectories(:,:,a), obj.interval);
            end
        end


        function obj = sim(obj)
            X = [];
            T = [];
            obj.updateEstimate();

            obj.estimate.value
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
                        obj.updateEstimate();
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
                exit = t(end) > 50;
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
                %
                start_ = start_ + sz;
            end
            figure()
            if obj.dim == 2
                plot(obj.estimate_history(:,1), obj.estimate_history(:,2),"r"); hold on;
                plot(obj.estimate_history(:,1), obj.estimate_history(:,3),"g");
            elseif obj.dim == 3
                plot(obj.estimate_history(:,1), obj.estimate_history(:,2),"r");hold on;
                plot(obj.estimate_history(:,1), obj.estimate_history(:,3),"g");hold on;
                plot(obj.estimate_history(:,1), obj.estimate_history(:,4),"b");hold on;
            end    
        end

        % this function is the dynamic equations of
        % all the agents
        function [xdot, start_] = f(obj, t, x)
            xdot = zeros(obj.n_agents*2*obj.dim + obj.extr_dim, 1);
            sz = 2*obj.dim;
            start_ = 1;

            for a=1:obj.n_agents
                agent = obj.agents(a);
                %
                xdot(start_:start_+sz-1) = agent.cl_dyn(t, x(start_:start_+sz-1));
                %
                start_ = start_ + sz;
            end

         
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

        % we shall not change the state of the environment here because it
        % gets called arbitrarily many times
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

