function [J,...
          link_ends,...
          link_end_set,...   
          link_end_set_with_base,...
          v_diff,...
          joint_axis_vectors,...
          joint_axis_vectors_R,...
          R_links,...
          J_world] = arm_Jacobian_in_link_frame(link_vectors,joint_angles,joint_axes,link_number)
% Construct a translation-rotation Jacobian for a chain of links as a
% function of the link vectors, the joint angles, joint axes, and the
% number of the link whose endpoint is the location where the Jacobian is
% evaluated, *and expressed in the frame of the link*
%
% Inputs:
%
%   link_vectors: A 1xn cell array in which each entry is a 3x1 link vector
%   joint_angles: a nx1 vector, each element of which is the joint angle
%       preceeding the corresponding link
%   joint_axes: a nx1 cell array, each entry of which is 'x','y', or 'z',
%       designating the axis of the corresponding joint
%   link_number: The number of the link whose Jacobian we want to evaluate
%
% Output:
%
%   J: The translation-rotation Jacobian for the end of link 'link_number',
%      *expressed in the frame of the link*. This Jacobian should be taken
%       with respect to *all* joint angles (and all columns of J
%       corresponding to joints after the link should be zeros)
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
%   R_links: The orientations of the individual links
%   J_world: The Jacobian as it appears in the world frame

    % First, use arm_Jacobian_with_rotation to get the Jacobian and all its
    % intermediate values. Call the Jacobian output of this function
    % 'J_world'
    [J_world,...
    link_ends,...
    link_end_set,...   
    link_end_set_with_base,...
    v_diff,...
    joint_axis_vectors,...
    joint_axis_vectors_R,...
    R_links,...
    J_translational] = arm_Jacobian_with_rotation(link_vectors,joint_angles,joint_axes,link_number);
      
    % Second, find the inverse of the orientation matrix of link
    % 'link_number' by taking its transpose. Use .' instead of ' (this
    % makes symbolic calls to the function use the true transpose instead
    % of the complex-conjugate transpose). Call this matrix 'R_inv'
    R_inv = R_links{link_number}.';

    % Third, make a matrix that has the block structure [R_inv Z; Z R_inv]
    % (where Z is a matrix of zeros the same size as R_inv) and multiply
    % this matrix by 'J_world' to produce the Jacobian 'J' in the frame of
    % the link.
    Z = zeros(size(R_inv));
    J = [R_inv Z; Z R_inv] * J_world;
      
end