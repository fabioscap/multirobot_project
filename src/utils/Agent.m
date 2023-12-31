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
        dyn_size
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

            % position and velocity
            obj.dyn_size = 2;
            obj.cl_dyn = @(t,x) doubleInt(t, x, obj.PDffwBrt(t, x));

        end
        
        function obj = updateReference(obj, p, int)
            obj.cl_dyn = @(t,x) doubleInt(t, x, obj.PDffwBrt(t, x));
            % precompute the derivatives for efficiency
            obj.int = int;
            obj.p = p;
            pd_ = diffBernstein(p, int);
            obj.pd = pd_;
            pdd_ = diffBernstein(pd_, int);
            obj.pdd = pdd_;
        end

        % a controller that follows a bernstein polynomial
        % trajectory through PD + ffw
        function u = PDffwBrt(obj, t, x)
            
            Kp = 100;
            Kd = 10;
            
            pos = x(1:obj.dim);
            vel = x(obj.dim+1:end);
            if t>=obj.int(1) && t <= obj.int(end)
            u = Kp*(deCasteljau(obj.p, t, obj.int)-pos) + ...
                Kd*(deCasteljau(obj.pd,t, obj.int)-vel) + ...
                deCasteljau(obj.pdd, t, obj.int);
            else 
            u = Kp*(deCasteljau(obj.p, obj.int(end), obj.int)-pos) + ...
                Kd*(0-vel) + ...
                0;
            end
        end
    end
end

