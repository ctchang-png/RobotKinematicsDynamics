function R = Rx(psi)
% Rotation matrix about the x axis
%
% Input:
%   
%   psi: scalar value for rotation angle
%
% Output:
%
%   R: 3x3 rotation matrix that for rotation by psi around the x axis
R = [1, 0, 0;...
     0, cos(psi), -sin(psi);...
     0, sin(psi),  cos(psi)];

end
