function v_set_extended = vector_set_extend(v_set,v_extensions)
% Takes a set of vectors and a set of lengths by which to extend them.
% Returns a set of extended vectors
%
% Inputs: 
%
%   v_set: a cell array, each element of which is a vector
%
%   v_extensions: a vector of the same dimensions as v_set, containing
%       lengths which should be added to the vectors
%
% Outputs:
%
%   v_set_extended: cell array of the same size as v_set, each element of
%       which is the vector with its new length

    %%%%%%%%
    % Use the 'cell' and 'size' commands to create the output
    % v_set_extended as an empty cell of the same size as v_set
    v_set_extended = cell(size(v_set));
    %%%%%%%%
    % Loop over the vectors and extensions, adding the extension to the
    % length of the vector, and then storing the result in the
    % corresponding entry of v_set_extended
    for idx = 1:numel(v_set_extended)
        v = v_set{idx};
        v_set_extended{idx} = (norm(v) + v_extensions(idx)) * v / norm(v);
    end


end