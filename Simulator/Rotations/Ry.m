function R = Ry(phi)
% Rotation matrix about the y axis
%
% Input:
%   
%   phi: scalar value for rotation angle
%
% Output:
%
%   R: 3x3 rotation matrix that for rotation by phi around the y axis
    R = [cos(phi), 0, sin(phi);...
         0, 1, 0;...
         -sin(phi), 0,  cos(phi)];

end