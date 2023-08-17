classdef EnvironmentDec < Environment
    % Decentralized environment class
    properties
        % adjacency matrix of the agents
        A
        % laplacian matrix
        L


    end
    methods

        function obj = EnvironmentDec(dim, N, p_t, tau)
            obj@Environment(dim, N, p_t, tau);
            
            obj.extr_dim = obj.extr_dim + obj.dim *obj.n_agents;
            
            % we need to define a connection for the agents
            obj.A = circleConnection(obj.n_agents);
            obj.L = adj2laplacian(obj.A);
        end
        % this function is the dynamic equations of
        % all the agents and the consensus dynamics
        function [xdot, start_] = f(obj, t, x)
            [xdot, start_] = f@Environment(obj, t, x);
            % this is where you would add consensus on
            % the individual agent estimates
            % it should look something like 
            xdot(start_:obj.dim *obj.n_agents) = -obj.L*x(start_:obj.dim*obj.n_agents);
            
            % we return also start_ in case we want to subclass
            % EnvironmentDec further (we don't)
            start_ = start_ + obj.dim *obj.n_agents;
        end
         
        % We need to modify these two functions because in a decentralized
        % environment each agent executes its own routines
        % RLS recursive step (eq. 5)
        function obj = updateEstimate(obj)
            % sig_norms = zeros(obj.n_agents,1);
            % for i=1:obj.n_agents
            %     p = obj.positions(:,1,i);
            %     sig_norms(i) = getARTVAsig(p, obj.p_t, eye(3), eye(3), false, obj.m);
            % end
        end

        % trajectory planning
        function obj = planTrajectories(obj)
            for a=1:obj.n_agents
                obj.agents(a).planTrajectory(obj.positions(:,:,a), obj.t);
            end
        end


    end
end

