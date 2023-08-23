classdef Estimate < handle
    %ESTIMATE 
    
    properties
        % the actual estimate value
        value
        x
        dim_x
        
        
        % coefficients for the approximation of the output
        m % artva amplitude
        ab = eq8(200);


        % RLS parameters
        S ;
        % forgetting factor
        beta = 0.8;

    end
    
    methods

        function obj = Estimate(dim, m)
            obj.m = m;
            if dim == 2
                obj.dim_x = 6;
                obj.x = [1 0 1 -1 1 0]';
            elseif dim == 3
                obj.dim_x = 10;
                obj.x = [1 0 0 1 0 1 0 0 0 0]';
            end
            obj.S =0.1* eye(obj.dim_x, obj.dim_x);

        end
        % RLS recursive step (eq. 5)
        % RLS recursive step (eq. 5)
        function obj = RLSStep(obj, positions, sig_norms)
            % we update the estimate of the unknown vector x
            % this has to be called whenever we receive 
            % a new ARTVA sample

            n_agents = size(sig_norms, 1);

            Y = zeros(n_agents,1);
            H = zeros(obj.dim_x, n_agents);
            for i=1:n_agents
                p = positions(:,1,i);
                % construct the output
                Y(i) = ( (obj.m/(sig_norms(i)*4*pi))^(2/3) )* (obj.ab(1)*obj.ab(2))^2;
                % build phi vector
                H(:,i) = buildPhi(p);
            end

            K = obj.S * H /( obj.beta*eye(n_agents, n_agents) + H'*obj.S*H) ;
            obj.x = obj.x + K *(Y - H'*obj.x);
            obj.S = (1/obj.beta) * (obj.S - K*H'*obj.S);

            obj.value = extractTarget(obj.x, obj.ab(1), obj.ab(2));

            obj.value
        end
 
    end
end

