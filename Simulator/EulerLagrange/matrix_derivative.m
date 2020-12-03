function [dM_function,...
            M_sym,...
            M_sym_column,...
            dM_sym_column,...
            dM_column_function] = matrix_derivative(M_function,n_parameters)
% Produce a matlab function that is the derivative of a matrix
% with respect to n configuration variables.
%
% Inputs:
%
%   M_function: The handle to a function that takes in a vector of
%       parameter values and outputs a matrix
%   n_parameters: The number of joints in the system
%
% Outputs:
%
%   dM_function: A function that takes in a vector of parameter values and
%       outputs a 1xn cell array whose entries are the derivatives of the
%       input matrix with respect to the corresponding parameters at
%       the configuration defined by the set of parameter values

    % Create a symbolic nx1 vector called 'configuration' using "sym('c',[n_parameters 1]);"
    configuration = sym('c', [n_parameters 1]);
    % Define the values of the configuration variables as being real
    % numbers using 'assume(configuration,'real');'
    assume(configuration, 'real')
    % Evaluate the matrix function with the symbolic configuration vector
    % as input. Call this output 'M_sym'
    M_sym = M_function(configuration);
    
    % Convert the symbolic matrix to a column called 'M_sym_column'
    M_sym_column = M_sym(:);
    % Take the derivative of M_sym_column with respect to the configuration
    % variables using the 'jacobian' function, and call this output
    % 'dM_sym_column'
    dM_sym_column = jacobian(M_sym_column, configuration);
    
    % Convert dM_sym_column into a matlab function using the
    % command 'dM_column_function = matlabFunction(dM_sym_column,'var',{configuration});'
    % and call the output 'dM_column_function'
    dM_column_function = matlabFunction(dM_sym_column, 'var', {configuration});
    % Create a function (using the @ syntax) that nests dM_column_function inside
    % columns_to_cells, so that the combined function takes in a vector of
    % parameter values and outputs a cell array, each entry of which is the
    % derivative of the matrix with respect to one of the parameters. Call
    % this function 'dM_function'
    dM_function = @(configuration) columns_to_cells(dM_column_function(configuration), size(M_sym));
    
end
