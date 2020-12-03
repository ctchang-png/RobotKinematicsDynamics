function v_set_s = vector_set_reverse_cumulative_sum(v_set,v_number)
% Take a set of vectors and an number v_number, and return a set of vectors
% in which each element is the cumulative sum of vectors between that
% element of the original set and the element v_number
%
% Inputs:
%
%   v_set: a 1xn cell array, each element of which is a 2x1 or 3x1 vector
%
%   v_number: the number that should be the last vector in each sum
%
% Output:
%
%   v_set_s: a 1xn cell array, the jth element of which is a 2x1 or 3x1 vector,
%       calculated as the sum of vectors in v_set from j to v_number

    %%%%%%%%
    % Start by making an empty cell array of the same size as v_set;
    
    %%%%%%%%
    % Loop over v_set_s
        
        % Put a zero vector into v_set_s{idx} (of the same size as the
        % vectors in v_set) as a default value 
        
        % Make a matrix whose columns are the elements of v_set from idx
        % through v_number. (cell2mat is useful here)
        
        


end
