function s_set_R = surface_set_rotate(s_set, R_set)
% Rotate a set of vectors specified in local coordinates by a set of
% rotation matrices that specifies the orientations of the frames in which
% the surfaces are defined.
%
% Inputs:
%
%   s_set: a 1xn cell array, 
%       each element of which is itself a cell array whose elements are a
%       set of matrices containing the X, Y, and Z coordinates of points on
%       a surface in local coordinates
%   R_set: a 1xn cell array, each element of which is a 3x3 rotation
%       which defines the orientations of the frames in which the surfaces
%       from s_set are defined
%
% Output:
%
%   s_set_R: a 1xn cell array, each element of which is a a 3x1 cell array,
%       each element of which is itself a cell array whose elements are a
%       set of matrices containing the X, Y, and Z coordinates of points on
%       the rotated surfaces

    %%%%%%%%%%%%
    % Copy s_set into a new variable named 's_set_R';
    s_set_R = s_set;
    
    %%%%%%%%%%%%%
    % Loop over the set of surfaces, rotating each surface by the
    % corresponding rotation matrix (using the 'rotate_surface' function)
    for idx = 1:numel(s_set)
        s_set_R{idx} = rotate_surface(s_set{idx}, R_set{idx});
    end


end