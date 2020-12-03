function [J,...
          link_ends,...
          link_end_set,...   
          link_end_set_with_base,...
          v_diff,...
          joint_axis_vectors,...
          joint_axis_vectors_R,...
          R_links,...
          J_translational] = arm_Jacobian_with_rotation(link_vectors,joint_angles,joint_axes,link_number)
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
%       with respect to *all* joint angles (all columns of J corresponding
%       to joints after the link should be zeros) *and expressed in the j.
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
%   J_translational: The translational component of the Jacobian

    % First, use arm_Jacobian to get the translation Jacobian for link
    % 'link_number' and the 'joint_axis_vectors_R' for the chain. Call the
    % Jacobian calculated here 'J_translational'
    [J_translational,...
          link_ends,...
          link_end_set,...   
          link_end_set_with_base,...
          v_diff,...
          joint_axis_vectors,...
          joint_axis_vectors_R,...
          R_links] = arm_Jacobian(link_vectors,joint_angles,joint_axes,link_number);     
    
    % Second, convert joint_axis_vectors_R from a cell array of vectors to a
    % matrix in which each vector is a column, using the [A{:}] syntax.
    % Call this matrix 'J_rotational'
    J_rotational = [joint_axis_vectors_R{:}];
    
    % Third, build the complete Jacobian by stacking the translational
    % component on top of the rotational component, using the [A;B] syntax.
    % Call this complete Jacobian 'J'
    J = [J_translational;J_rotational];

end