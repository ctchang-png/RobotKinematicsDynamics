function [J,...
% Construct a Jacobian for a chain of links as a function of the link
% vectors, the joint angles, joint axes, and the number of the link whose
% endpoint is the location where the Jacobian is evaluated
%
% Inputs:
%
%   link_vectors: A 1xn cell array in which each entry is a 3x1 link vector
%   joint_angles: a nx1 vector, each element of which is the joint angle
%       preceeding the corresponding link
%   joint_axes: a nx1 cell array, each entry of which is 'x','y', or 'z',
%       designating the axis of the corresponding joint
%   link_extensions: The amounts by which the lengths of the prismatic
%       links should be extended
%   prismatic: a vector of 0 and 1 values that says which links are
%       actually prismatic
%   link_number: The number of the link whose Jacobian we want to evaluate
%
% Output:
%
%   J: The Jacobian for the end of link 'link_number', with respect to
%       *all* joint angles (All columns of J corresponding to joints after
%       the link should be zeros)
%
% Additional outputs (These are intermediate variables. Having the option
%   to return them as outputs lets our automatic code checker tell you
%   where problems are in your code):
%
%   link_end_set_with_base: The endpoints of the links after taking the cumulative
%       sum of link vectors, with a zero point added at the beginning
%   v_diff: The set of vectors from all points in 'link_end_set_with_base'
%       to the end of link 'link_number'
%   joint_axis_vectors: The unit vectors pointing along each joint axis in
%       local coordinates
%   joint_axis_vectors_R: The unit vectors pointing along each joint axis
%       in world coordinates


    % Use 'threeD_robot_arm_endpoints' to get 'link_ends', 'R_links',
    % 'link_end_set' and 'link_end_set_with_base'. Note that you won't need the
    % other outputs from that function, so you can write ~ for those outputs
    % instead of creating a variable that you won't use
    %
    % For example, '[~,output_2] = f(x)' makes f(x) act as if it was asked to provide
    % two outputs, but only creates 'output_2'. Using this syntax makes it
    % easier for Matlab's code checking to tell you if you have a bug in which
    % you are forgetting to use a variable that you have defined (which
    % generally means that you're using the wrong variable somewhere)



    % Use 'v_diff' to get the vector to the end of link 'link_number' from each
    % point in link_end_set_with_base

    % Use 'threeD_joint_axis_set' to turn the joint angles and joint axes into
    % a set of axis vectors called 'joint_axis_vectors'
    
    % Use 'vector_set_rotate' to rotate the joint axes by the link
    % orienations (Note that although our convention is that the ith joint
    % is in the (i-1)th link, the vector associated with the joint axis is
    % the same in both frame (i-1) and frame i, so we can rotate the joint
    % axes directly into the corresponding frames (this means we don't have
    % to offset the joint axes when calling 'vector_set_rotate'). Call the
    % output of this rotation 'joint_axis_vectors_R'.


    % Create a zero matrix to hold the Jacobian. It should have three rows,
    % and should have as many columns as there are joints

    % Fill in the columns of the Jacobian. Each column is either:
    %
    %   A. The cross product of the corresponding joint axis with the vector
    %   from that joint location to the end of link 'link_number', or
    %
    %   B. Zero, if the joint is after the end of link 'link_number'
    %
    % (Note that you can implment this conditional by making the loop that
    % fills in the columns of J stop at column 'link_number')

    
end
