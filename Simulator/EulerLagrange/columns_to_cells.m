function cell_output = columns_to_cells(matrix_input,output_size)
% Take in a matrix and return a cell array in which each entry of the cell
% array corresponds to one column of the input matrix, reshaped into the
% dimensions described by the second argument to the function
%
% Inputs:
%
%   matrix_input: An nxm matrix
%   output_size: A vector containing the dimensions into which the columns
%       of the matrix input should be reshaped. The product of these
%       dimensions should be equal to n (the number of rows in
%       matrix_input)
%
% Output:
%
%   cell_output: An nx1 cell array, each element of which is a matrix
%       containing the corresponding column of matrix_input, reshaped to be
%       of the dimensions in output_size


    % First, create a 1xn cell array called 'cell_output' with as many
    % elements as there are columns of 'matrix_input'
    cell_output = cell(1, size(matrix_input, 2));
    
    % Loop over the columns of 'matrix_input', reshaping them into the
    % dimensions specified in 'output_size' and then storing them in the
    % corresponding entries of 'cell_output'
    for idx = 1:numel(cell_output)
        tmp = matrix_input(:,idx);
        cell_output{idx} = reshape(tmp, output_size);
    end


end