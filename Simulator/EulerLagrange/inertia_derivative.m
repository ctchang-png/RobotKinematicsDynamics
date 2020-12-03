function dM_function = inertia_derivative(M_function,n_joints)
% Produce a matlab function that is the derivative of the inertia matrix
% with respect to the n configuration variables.
%
% Inputs:
%
%   M_function: The handle to a function that takes in a vector of joint
%       angles and outputs the an inertia matrix
%   n_joints: The number of joints in the system
%
% Outputs:
%
%   dM_function: A function that takes in a vector of joint angles and
%       outputs a 1xn cell array whose entries are the derivatives of the
%       inertia matrix with respect to the corresponding joint angles at
%       the configuration defined by the set of joint angles

    % Create a symbolic nx1 vector called 'configuration'
    
    % Define the values of the configuration variables as being real
    % numbers
    
    % Evaluate the inertia matrix with the symbolic configuration vector as
    % input
    
    
    % Convert the symbolic inertia matrix to a column
    
    % Take the derivative of M_sym_column with respect to the configuration
    % variables
    
    
    % Convert dM_sym_column to a matlab function
    
    % Create an anonymous function that nests dM_column_function inside
    % columns_to_cells, so that the combined function takes in a vector of
    % joint angles and outputs a cell array, each entry of which is the
    % derivative of the mass matrix with respect to one of the joint angles
    
end
