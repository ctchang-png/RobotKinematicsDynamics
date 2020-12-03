function [state_velocity,...
          configuration,...
          velocity,...
          acceleration] = EulerLagrange_trajectory(time,state,M_function,dM_function,F_function)
% Follow a trajectory whose acceleration is determined by an inertia matrix
% and forcing function
%
% Inputs:
%
%   time: A scalar value describing the time at which the system dynamics
%       are evaluated
%   state: A (2n)x1 vector, whose first n entries are the system's current
%       configuration (e.g., the joint angles) and whose remaining entries
%       are the system's current configuration velocity (e.g., the joints'
%       angular velocities)
%   M_function A handle to a function that takes in a configuration
%       vector and returns an nxn cell array, encoding the inertia matrix
%       as a function of the configuration (e.g., the joint angles).
%   dM_function: A handle to a function that takes in a configuration
%       vector and returns a 1xn cell array, whose entries are the
%       derivative of the inertia matrix with respect to configuration
%       (e.g., the joint angles).
%   F_function: A handle to a function that takes in a configuration
%       vector, configuration velocity, and time, and returns an nx1 vector
%       of configuration forces (e.g., the torques around the joints).
%
% Output:
%
%   state_velocity: A (2n)x1 vector, whose first n entries are the system's
%       current configuration velocity (e.g., the joints' angular
%       velocities), and whose remaining entries are the system's
%       configuration acceleration (e.g., the joints' angular
%       accelerations)
%
% Additional output:
%
%   configuration: The first half of the state vector
%   velocity: The second half of the state vector
%   acceleration: The configuration acceleration found by evaluating the
%       Euler-Lagrange equation

    % Create two vectors, 'configuration' and 'velocity', that respectively
    % contain the first and second halves of the state
    n = size(state, 1) / 2;
    configuration = state(1:n);
    velocity = state(n+1:end);
    % Use 'EulerLagrange_acceleration' to find the acceleration of the
    % system given its current configuration, velocity, and force. Save
    % this vector in a variable named 'acceleration'
    [acceleration,...
    M,...
    C,...
    F]= EulerLagrange_acceleration(M_function,dM_function,F_function,time,configuration, velocity);
    
    % Build the 'state_velocity' vector by stacking the system velocity on
    % top of the system acceleration
    state_velocity = [velocity; acceleration];
end
    
    