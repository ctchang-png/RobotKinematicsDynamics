function [xyz] = circle_x(t)
% Generate points on a unit circle in the y-z plane (with the positive x
% axis passing perpendicularly through the center of the circle), where the
% range of t=[0 1] corresponds to one full cycle of the circle
%
% Input:
%
%   t: 1xn or nx1 vector of times at which to generate points on the circle
%
% Output:
%
%   xyz: 3xn matrix. Each row is the x, y, or z location of points on the
%       circle, and each column corresponds to one time point from the
%       input

    % First, make t a 1xn vector if it isn't already one. Use 't(:)' to
    % turn t into a column, and then use the ' operation to make it a row
    t = reshape(t, 1, numel(t));
    % Second, use 'zeros', 'sin', and 'cos' operations to convert t into
    % xyz points. Don't forget to use a 2*pi factor so that the period of
    % the circle is 1. Save the output as variable 'xyz'
    xyz = [zeros(size(t));...
           sin(2*pi*t);...
           cos(2*pi*t)];
end