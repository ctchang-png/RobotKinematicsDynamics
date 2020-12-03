function R = Rz(theta)
% Rotation matrix about the z axis
%
% Input:
%   
%   theta: scalar value for rotation angle
%
% Output:
%
%   R: 3x3 rotation matrix that for rotation by theta around the z axis
R = [cos(theta), -sin(theta), 0;...
     sin(theta),  cos(theta), 0;...
     0, 0, 1];

end
