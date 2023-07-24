classdef Environment < handle
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


        tau % ARTVA sampling time
        m=10   % ARTVA amplitude
        
        % coefficients for the approximation of the output
        ab = eq8(200); 

        % TODO validate these parameters
        % initializer for the S matrix in RLS
        S ;
        % forgetting factor
        beta = 0.8;


        % simulation parameters
        t = 0 % simtime
        dt = 0.001 % timestep
        T= 100 % total simulation time
        % TODO add stopping criteria
        
        % trajectory interval (you get this from solving opt problem)
        interval

        % replanning interval
        t_bar = 10;


    end
    
    methods
        function obj = Environment(dim, N, p_t, tau)
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
                disp("agent "+i+" at pos: ");
                random_position

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

            obj.p_hat = zeros(dim, 1);

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

        % trajectory planning
        function obj = planTrajectories(obj)
            [obj.interval, obj.trajectories] = planningProblem(obj);
        end


        function obj = sim(obj)
            disp("sim")
            obj.t = 0;
            % plan the first trajectories
            obj.planTrajectories();
            last_replan = 0;
            
            samples = 0;

            for t=linspace(0, obj.T, obj.T/obj.dt)
                obj.t = t;

                % check if we have a new ARTVA sample
                if (floor(t/obj.tau) >= samples)
                    samples = samples+1;
                    % refine the estimation
                    obj.RLSStep();
                    %pause();
                end

                % if conditions are met replan
                if t - last_replan > obj.t_bar || t > obj.interval(end)
                    disp("replanning at " + t)
                    obj.planTrajectories();
                    last_replan = t;
                end


                % update position of agents
                for a=1:obj.n_agents
                    obj.positions(:,1,a) = ...
                        deCasteljau(obj.trajectories(:,:,a), t, obj.interval);
                end
                
            end

        end
    end
end

