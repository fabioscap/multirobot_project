classdef EnvironmentDec < Environment
    % Decentralized environment class
    properties
        % adjacency matrix of the agents
        A
        % laplacian matrix
        L
        
        L_expanded

    end
    methods
        function obj = EnvironmentDec(dim, N, p_t, tau)
            obj@Environment(dim, N, p_t, tau);
            
            obj.extr_dim = obj.extr_dim + obj.dim *obj.n_agents;
            
            % we need to define a connection for the agents
            obj.A = circleConnection(obj.n_agents);
            obj.L = adj2laplacian(obj.A);
            obj.L_expanded = expandLaplacian(obj.L, obj.dim);
        end

        % this function is the dynamic equations of
        % all the agents and the consensus dynamics
        function [xdot, start_] = f(obj, t, x)
            xdot = zeros(obj.n_agents*obj.dyn_size*obj.dim + obj.extr_dim, 1);
            sz = obj.dyn_size*obj.dim;
            start_ = 1;
            

            % DYNAMICS: consensus part
            consensus_idx = obj.n_agents*obj.dyn_size*obj.dim;

            % get the part that is related to consensus (p_hat)
            x_cons = x(consensus_idx+1: ...
                       consensus_idx + obj.extr_dim);

            % p_hat_dot
            cons_action = -obj.L_expanded * x_cons;

            % p_hat_dot = -L * p_hat
            xdot(consensus_idx+1: ...
                 consensus_idx + obj.dim*obj.n_agents) = cons_action;
            
            % DYNAMICS: agent part

            % start at 0 for mental sanity
            for a=0:obj.n_agents-1
                agent = obj.agents(a+1);
                %
                % get the position (p_hat) and velocity (p_hat_dot)
                % references for the agent
                p_target = x_cons(obj.dim*a + 1: obj.dim*(a+1));
                pd_target = cons_action(obj.dim*a + 1: obj.dim*(a+1));
                xdot(start_:start_+sz-1) = agent.ol_dyn(t, x(start_:start_+sz-1), ...
                    p_target, ...
                    pd_target, ...
                    zeros(obj.dim, 1)); % no feed forward
                %
                start_ = start_ + sz;
            end

        end
        
        % environment each agent executes its own routines
        % RLS recursive step (eq. 5)
        function obj = updateEstimate(obj)
            sig_norms = zeros(obj.n_agents,1);
            for i=1:obj.n_agents
                p = obj.positions(:,1,i);
                sig_norms(i) = getARTVAsig(p, obj.p_t, eye(3), eye(3), false, obj.m);
            end

            for i=1:obj.n_agents
                neighbor_indexes = find(obj.A(i,:));

                sig_norms_available = sig_norms([i, neighbor_indexes]);
                positions_available = obj.positions(:,:,[i, neighbor_indexes]);

                obj.agents(i).updateEstimate(positions_available, ...
                                             sig_norms_available);
            end
        end


        function [T, X] = sim(obj)
            X = [];
            T = [];
            obj.updateEstimate();

            % set up the events
            options = odeset("Events", @(t, y) obj.eventsFcn(t, y));

            % build the initial condition vector
            sz = obj.dyn_size*obj.dim;
            start_ = 1;
            x0 = zeros(1, obj.n_agents*obj.dyn_size*obj.dim + obj.extr_dim);
                        
            consensus_idx = obj.n_agents*obj.dyn_size*obj.dim;
            for a=0:obj.n_agents-1
                agent = obj.agents(a+1);
                %
                x0(start_:start_+sz-1) = agent.x;
                %
                start_ = start_ + sz;

                %x0(consensus_idx + obj.dyn_size*a+1:...
                   %consensus_idx + obj.dyn_size*(a+1)) = agent.estimate.value;
                x0(consensus_idx + obj.dim*a+1:...
                   consensus_idx + obj.dim*(a+1)) = agent.estimate.value;
            end


            total_sim_time = 150;
            exit = false;
            while ~exit
                disp("Integrating from "+ obj.t + " to " + total_sim_time);
                [t, x, te, xe, ie] = ode45(@(t,x) obj.f(t,x), [obj.t, total_sim_time], x0, options);
               
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
                    exit = true;
                    break

                else % event
                    % we set obj.position so we apply the updates
                    % at the correct position
                    sz = 2*obj.dim;
                    start_ = 1;
                    for a=1:obj.n_agents
                        %
                        obj.positions(:,:,a) = x(end,start_:start_+...
                            (obj.dim-1));
                        %
                        start_ = start_ + sz;
                    end
                    if ie == 1
                        x0 = xe;
                        obj.t = te;
                        % do RLS...
                        obj.updateEstimate();
                        for a=0:obj.n_agents-1
                            agent = obj.agents(a+1);
                            x0(consensus_idx + obj.dim*a+1:...
                               consensus_idx + obj.dim*(a+1)) = agent.estimate.value;
                        end
                        %
                        obj.last_sample = te;
                    else
                        % this should not happen
                        error("what?")
                    end
                end
                exit = exit || t(end) > total_sim_time;
            end
            
            size(t)
            size(X)

            % TODO move this away
            % plot the positions of the agents in time
            f = figure();
            if obj.dim == 2
                scatter(obj.p_t(1), obj.p_t(2), "cyan","Marker", "o",...
                    'LineWidth', 2); hold on
            elseif obj.dim == 3
                scatter3(obj.p_t(1),obj.p_t(2),obj.p_t(3),"cyan",...
                    "Marker", "o", 'LineWidth', 2); hold on
            else
                error("cannot plot this dimension")
            end
            sz = 2*obj.dim;
            start_ = 1;
            for a=1:obj.n_agents
                agent = obj.agents(a);
                %
                if obj.dim == 2
                    plot(X(:,start_), X(:,start_+1),"b");
                    plot(X(end,start_), X(end,start_+1),"r",'Marker','x','LineWidth',2)
                    hold on;
                elseif obj.dim == 3
                    plot3(X(:,start_), X(:,start_+1),X(:,start_+2),"b");
                    plot3(X(end,start_),X(end,start_+1),X(end,start_+2),...
                        "r",'Marker','x','LineWidth',2)
                    hold on;
                else
                    error("cannot plot 3d yet")
                end

                %
                start_ = start_ + sz;
            end
            % plot consensus evolution
            %lines = ["-r","-b","--r","--b","-.r","-.b",":r",":b","-xr","-xb"];
            f=figure();
            for a=0:obj.n_agents-1
                agent = obj.agents(a+1);
                %
                if obj.dim == 2
                    plot(T, X(:,consensus_idx+obj.dyn_size+1),'r'); hold on;
                    plot(T, X(:,consensus_idx+obj.dyn_size*(a+1)),'c'); hold on;
                    plot(T, X(:,consensus_idx+obj.dyn_size*(a+1)),'g'); hold on;
                elseif obj.dim ==3
                    plot(T, X(:,consensus_idx+obj.dim*a+1),'r'); hold on;
                    plot(T, X(:,consensus_idx+obj.dim*a+2),'c'); hold on;
                    plot(T, X(:,consensus_idx+obj.dim*a+3),'g'); hold on;
                    legend('X','Y','Z')

                else
                    error("cannot plot 3d yet")
                end

                %
                start_ = start_ + sz;
            end
        end


        function [value, isterminal, direction] = eventsFcn(obj, t, y)
            % new ARTVA signal?
            value(1) = t - obj.last_sample > obj.tau;
            % we want to stop the integration when this happens
            isterminal = 1;
            % direction does not matter in our case
            direction(1) = 0;

            % TODO check if an agents wants to stop the integration
        end
    end
    
end

