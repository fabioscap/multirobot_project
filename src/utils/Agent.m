classdef Agent
    %AGENT
    % we assume an agent tracks perfectly the trajectory 
    % in position, so it has no dynamics
    
    % an agent has: 
    % - a state (position)
    % - a trajectory that gets updated through replanning
    %   it is parametrized by Bernstein polynomials

    % an agent does not have an orientation
    % (no mention in Optimal Motion Planning ...)

    properties
        dim % 2D or 3D
        position
        traj
    end
    
    methods
        function obj = Agent(initial_position)
            %AGENT Construct an instance of this class
            obj.position = initial_position;

            obj.dim = length(initial_position);
            obj.traj = zeros(dim,1);
        end
    end
end