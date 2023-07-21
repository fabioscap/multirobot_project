classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dim
        dim_x

        N % the number of agents
        p_t % the pose of the target (unknown to agents)
        x % the unknown vector x (see references)
        p_hat % the current estimate of the target
        x_bnd = [-50; 50];
        y_bnd = [-50; 50];
        z_bnd = [-10; 10];
        agents = []

        tau % ARTVA sampling time
        m=1   % ARTVA amplitude
        
        % coefficients for the approximation of the output
        ab = eq8(200); 

        % TODO validate these parameters
        % initializer for the S matrix in RLS
        S ;
        % forgetting factor
        beta = 0.9;


        % simulation parameters
        t=0;
        dt = 0.001 % timestep
        
        % trajectory horizon (you get this from solving opt problem)
        tf;

    end
    
    methods
        function obj = Environment(dim, N, p_t, tau)
            %ENVIRONMENT Constructor

            obj.dim = dim;

            obj.N = N;
            obj.p_t = p_t;
            
            for i=1:N
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
                ai = Agent(random_position);

                disp("agent "+i+" at pos: ");
                random_position
                if i == 1
                    obj.agents = [ai];
                else
                    obj.agents(i) = ai;
                end
            end
            
            obj.tau = tau;

            if dim == 2
                obj.dim_x = 6;
            elseif dim == 3
                obj.dim_x = 10;
            end
            obj.S = eye(obj.dim_x, obj.dim_x);

            obj.p_hat = zeros(dim, 1);

        end
        
        % RLS recursive step (eq. 5)
        function obj = RLSStep(obj)
            % we update the estimate of the unknown vector x
            % this has to be called whenever we receive 
            % a new ARTVA sample 
            Y = zeros(obj.N,1);
            H = zeros(obj.dim_x, obj.N);
            for i=1:obj.N
                p = obj.agents(i).position;

                % collect the individual measurements for each agent
                % TODO set noise to true
                sig = getARTVAsig(p, obj.p_t, false);
                % construct the output
                Y(i) = ( (obj.m/(norm(sig)*4*pi))^(1/3)*(obj.ab(1)*obj.ab(2)) )^2;

                % build phi vector
                H(:,i) = buildPhi(p);
            end
            obj.x = obj.x + obj.S\(H*(Y-H'*obj.x));
            obj.S = obj.beta*obj.S + H*H';
        end

        % trajectory planning
        function obj = planTrajectories(obj)
            % we put this in another file for readability

        end
    end
end

