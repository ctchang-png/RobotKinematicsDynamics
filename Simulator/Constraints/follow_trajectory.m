function [alpha_dot,v] = follow_trajectory(t,alpha,J,shape_to_draw)
% Find the set of 'alpha_dot' joint velocities at a time 't' and current
% joint angles 'alpha' that will make the end of a arm whose Jacobian
% function is 'J' follow the shape described by the function 'shape to
% draw'.
%
% Inputs:
%
%   t: 1x1 scalar value containing the time at which to evaluate the
%       velocity along the shape
%   alpha: nx1 vector of the arm's current joint angles
%   J: A function handle whose function maps the current joint angles to
%       the Jacobian of the end of the arm
%   shape_to_draw: A function handle whose function maps a set of m times
%       to a 3xm matrix whose columns are the xyz points of a shape in
%       3D space
%
% Output:
%
%   alpha_dot: The set of joint velocities that, when passed through 'J'
%       produce a vector equal to d/dt (shape_to_draw) evaluated at 't'
%   v: The derivative of 'shape_to_draw' at time 't'

    % First, use the 'jacobianest' function to take the derivative of
    % 'shape_to_draw' at time 't'. Save this to a variable named 'v'
    v = jacobianest(shape_to_draw, t);
    % Use 'lsqminnorm' with 'J', 'alpha', and 'v' to find the alpha_dot
    % that comes closest to producing 'v' when passed through 'J'
    alpha_dot = lsqminnorm(J(alpha), v);
    


end