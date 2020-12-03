function v_set_s = vector_set_cumulative_sum(v_set)
% Take the cumulative sum of a set of vectors.
%
% Inputs:
%
%   v_set: a 1xn cell array, each element of which is a 2x1 or 3x1 vector
%
% Output:
%
%   v_set_s: a 1xn cell array, each element of which is a 2x1 or 3x1 vector
%       and is the cumulative sum of vectors from v_set

    %%%%%%%%
    % Start by copying v_set into a new variable v_set_s;
    v_set_s = v_set;
    %%%%%%%%
    % Loop over v_set_s, adding each vector to the next vector
    for idx = 1:numel(v_set_s)
        if idx == 1
            continue
        end
        v_set_s{idx} = v_set_s{idx} + v_set_s{idx-1};
    end
        


end
