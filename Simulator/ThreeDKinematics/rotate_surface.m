function s_out = rotate_surface(s_in,R)
% Rotate a set of points defining a surface by a rotation matrix R. Points
% defining a surface are arranged as set of three matrices, and so cannot
% be directly operated on with a rotation matrix. The three options are
%
%   1. Write a for loop that extracts each set of xyz points from the
%       matrices, rotates them, and then puts them back in the
%       corresponding point in the matrices
%
%   2. Use Matlab's arrayfun command to do the same thing
%
%   3. Reshape the matrices into a single 3xn matrix, multiply the rotation
%       matrix into this matrix, and then reshape them back into three
%       matrices
%
% Inputs:
%
%   s_in: a 3x1 cell array, whose entries are a set of matrices containing
%       the X, Y, and Z coordinates of points on a surface
%
%   R: a 3x3 rotation matrix containing the matrix by which to rotate the
%       surface
%
% Output:
%
%   s_out: a 3x1 cell array, whose entries are a set of matrices containing
%       the X, Y, and Z coordinates of points on the surface after rotating
%       it

    %%%%%%%%%
    % Copy s_in into a new cell array named 's_out'
    s_out = s_in;

    %%%%%%%%%%%%%%
    % Loop over all points in the surface (each point is given by one entry
    % in each of the X, Y, and Z matrices stored in s_in).
    %
    %   1. Group the x, y, and z values for each point into a vector
    %   2. Rotate this vector with the rotation matrix
    %   3. Put the resulting x, y, and z values into the s_out X, Y, and Z
    %       matrices (at the same index-locations as they originally came
    %       from)
    for idx = 1:numel(s_in{1})
       
        % s_in{a}(b) extracts (b)th component of the {a}th cell  of s_in
        x = s_in{1}(idx);
        y = s_in{2}(idx);
        z = s_in{3}(idx);
        % Loop over all points in the rotated xyz vector, putting them back
        % into their correct places in the the s_out structure
        v_R = R * [x;y;z];
        s_out{1}(idx) = v_R(1);
        s_out{2}(idx) = v_R(2);
        s_out{3}(idx) = v_R(3);
    end

end