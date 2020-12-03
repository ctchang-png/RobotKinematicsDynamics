function [a,...
          M,...
          C,...
          F]= EulerLagrange_acceleration(M_function,dM_function,F_function,time,configuration, velocity)
% Use the Euler-Lagrange equation to calculate the coordinate acceleration
% of a system with a defined inertia matrix and subject to specified forces
%
% Inputs:
%
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
%   a: An nx1 vector of configuration accelerations (e.g., the joint
%       angular accelerations) corresponding to the system's current
%       configuration and velocity, and the forces acting on the system
%
% Additional output:
%
%   M: The current inertia of the system
%   C: The current centrifugal-Coriolis forces acting on the joints
%   F: The current forces acting on the joints




    % Evaluate the inertia matrix at the current joint angles, saving the
    % result as 'M'
    M = M_function(configuration);
    
    % Use 'CoriolisCentrifugal' to find the C vector corresponding to the
    % current configuration and velocity, saving the result as 'C'
    [C,...
    dM,...
    C_1_components,...
    C_1,...
    C_2] =...
    CoriolisCentrifugal(dM_function,configuration,velocity);
    
    % Evaluate the force acting on the system at the current time,
    % configuration, and velocity, saving the result as 'F'
    F = F_function(time, configuration, velocity);

    %%%%%%%%%%
    % Find the coordinate acceleration by subtracting C from F, and
    % multiplying the inverse of 'M' by the result of the subtraction.
    % (Here is a good place to use 'A\B' to implement 'inv(A)*B')
    a = M \ (F - C);

    
end