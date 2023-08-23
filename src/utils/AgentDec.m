classdef AgentDec < Agent
    properties
        % a decentralized agent keeps its own estimate
        estimate
        
        ol_dyn
        t
    end

    methods
        function obj = AgentDec(x0, m)
            obj@Agent(x0);
            obj.estimate = Estimate(obj.dim, m);
            
            obj.ol_dyn = @(t,x,p, pd, pdd) doubleInt(t, x, obj.PDffw(t, x, p, ...
                                                                pd, ...
                                                                pdd));
        end

        % a PD + ffw controller
        function u = PDffw(obj, t, x, p, pd, pdd)
            % the controller( different values for either 2D or 3D)
            if obj.dim ==2
                Kp = 100;
                Kd = 25;
            elseif obj.dim ==3
                Kp = 50;
                Kd = 45;
            end

            
            pos = x(1:obj.dim);
            vel = x(obj.dim+1:end);

            u = Kp*(p-pos) + ...
                Kd*(pd-vel) + ...
                pdd;
        end

        % update estimate of target using its own information
        % and the neighbors'
        function obj = updateEstimate(obj, P, Y)
            obj.estimate.RLSStep(P, Y);
        end

        % TODO we do not use bernstein for now in decentralized mode 
        % a decentralized agent has a function that replans its trajectory
        % function obj = planTrajectory(obj, pos, t)
        %     [new_p, new_int] = planningProblem(pos, ...
        %                                        t, ...
        %                                        obj.estimate.value, ...
        %                                        obj.tau);
        %     % make sure to update the new trajectories
        %     obj.updateReference(new_p, new_int);
        % 
        % end
    end
end

