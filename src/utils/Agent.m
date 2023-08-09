classdef Agent < handle
    %AGENT represents an agent
    % an agent has 
    % -dynamics equations 
    %  (assume double integrator for now)
    % -reference trajectory 
    %  (specified as bernstein polynomial)
    % -a controller that makes the agent follow
    %  the trajectory
    
    properties
        dim
        cl_dyn
        x
        % reference trajectories and derivatives
        p
        pd
        pdd
        int
    end
    
    methods
        function obj = Agent(x0)
            % x0 is the starting position
          
            obj.dim = length(x0);
            obj.x = zeros(2*obj.dim, 1);
            obj.x(1:obj.dim) = x0;
            obj.x(obj.dim+1:end) = 0;

            obj.cl_dyn = @(t,x) doubleInt(t, x, obj.PDffw(t, x));

        end
        
        function obj = updateReference(obj, p, int)
            % precompute the derivatives for efficiency
            obj.int = int;
            obj.p = p;
            pd_ = diffBernstein(p, int);
            obj.pd = pd_;
            pdd_ = diffBernstein(pd_, int);
            obj.pdd = pdd_;
        end

        function u = PDffw(obj, t, x)
            % the controller
            Kp = 100;
            Kd = 10;
            
            pos = x(1:obj.dim);
            vel = x(obj.dim+1:end);

            u = Kp*(deCasteljau(obj.p, t, obj.int)-pos) + ...
                Kd*(deCasteljau(obj.pd,t, obj.int)-vel) + ...
                deCasteljau(obj.pdd, t, obj.int);
        end
    end
end

