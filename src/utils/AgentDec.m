classdef AgentDec < Agent
    properties
        % a decentralized agent keeps its own estimate
        estimate
        tau
        m

        t


    end

    methods
        function obj = AgentDec(x0, tau, m)
            obj@Agent(x0);
            obj.tau = tau;
            obj.m   = m;
            obj.estimate = Estimate(obj.dim, obj.m);
           
        end

        % update estimate of target using its own information
        % and the neighbors'
        function obj = updateEstimate(obj, P, Y)
            obj.estimate.RLSStep(P, Y);
        end
        % a decentralized agent has a function that replans its trajectory
        function obj = planTrajectory(obj, pos, t)
            [new_p, new_int] = planningProblem(pos, ...
                                               t, ...
                                               obj.estimate.value, ...
                                               obj.tau);
            % make sure to update the new trajectories
            obj.updateReference(new_p, new_int);
            
        end
    end
end

