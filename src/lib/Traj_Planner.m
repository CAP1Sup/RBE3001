classdef Traj_Planner
    % Plans trajectories
    
    properties
        % None yet
    end
    
    methods
        function self = Traj_Planner()
        end
        
        % Calculates the cubic coefficients for a given transformation
        % Takes two matricies, an initial and a final
        % Each matrix is in form [t; x; v]
        function coefficients = cubic_traj(~, init, final)
            % Define the array positions for easier readability
            t = 1;
            x = 2;
            v = 3;

            % Build the maxtrices, then calculate the coefficients
            A = [1, init(t),    init(t)^2,    init(t)^3;
                 0, 1,        2*init(t),    3*init(t)^2;
                 1, final(t),   final(t)^2,   final(t)^3;
                 0, 1,        2*final(t),   3*final(t)^2;];
            B = [init(x); init(v); final(x); final(v)];
            coefficients = mldivide(A,B);
        end

        % Calculates the quintic coefficients for a given transformation
        % Takes two matricies, an initial and a final
        % Each matrix is in form [t; x; v, a]
        function coefficients = quintic_traj(~, init, final)
            % Define the array positions for easier readability
            t = 1;
            x = 2;
            v = 3;
            a = 4;

            % Build the maxtrices, then calculate the coefficients
            A = [1, init(t),    init(t)^2,    init(t)^3,     init(t)^4,     init(t)^5;
                 0, 1,        2*init(t),    3*init(t)^2,   4*init(t)^3,   5*init(t)^4;
                 0, 0,        2,            6*init(t),    12*init(t)^2,  20*init(t)^3;
                 1, final(t),   final(t)^2,   final(t)^3,    final(t)^4,    final(t)^5;
                 0, 1,        2*final(t),   3*final(t)^2,  4*final(t)^3,  5*final(t)^4;
                 0, 0,        2,            6*final(t),   12*final(t)^2, 20*final(t)^3;];
            B = [init(x); init(v); init(a); final(x); final(v); final(a)];
            coefficients = mldivide(A,B);
        end
    end % methods
end % Traj_Planner

