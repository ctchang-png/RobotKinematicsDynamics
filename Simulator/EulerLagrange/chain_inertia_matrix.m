function [M,...
          mu_links,...
          J,...
          M_links] = chain_inertia_matrix(link_vectors,joint_angles,joint_axes,link_radii)
% Construct an inertia matrix for a chain of links with respect to its
% joint angles as a function of the link vectors, the joint angles, joint
% axes, and the diameters of the links (taking the links as uniform
% cylinders).
%
% Inputs:
%
%   link_vectors: A 1xn cell array in which each entry is a 3x1 link vector
%   joint_angles: a nx1 vector, each element of which is the joint angle
%       preceeding the corresponding link
%   joint_axes: a nx1 cell array, each entry of which is 'x','y', or 'z',
%       designating the axis of the corresponding joint
%   link_radii: A nx1 vector containing the diameters of the links
%       perpendicular to their axes
%
% Output:
%
%   M: The nxn inertia matrix for the chain with respect to its joint
%       angles
%
%
% Additional output (This is an intermediate variable. Having the option
%   to return it as output lets our automatic code checker tell you
%   where problems are in your code):
%
%   mu_links: The inertia matrices for the links in their own frames
%   J: The translation-rotation Jacobians for the links.
%   M_links: The inertia matrices for the links with respect to the joints


    %%%%%%%%%%
    
    % Make a cell array 'mu_links' that is the same size as the
    % link_vectors cell array
    mu_links = cell(size(link_vectors));
    
    % Loop over the links in the chain, generating their inertia matrices
    % with 'link_inertia_matrix' and saving them in the corresponding
    % entries of mu_links
    for idx = 1:numel(link_vectors)
        mu_links{idx} = link_inertia_matrix(link_vectors{idx}, link_radii(idx));
    end

        
    
    %%%%%%%%%
    
    % Make a cell array 'J' that is the same size as the
    % link_vectors cell array
    J = cell(size(link_vectors));

    % Loop over the links, generating the Jacobians *to the link centers*
    % and saving them to the corresponding entry of J. Some of the comments
    % below assume that the loop variable name is 'idx'; this is a
    % suggestion and your code will not be marked wrong if you use a
    % different loop variable name.
    for idx = 1:numel(link_vectors)
        
        % First, make a copy of the 'link_vectors' cell array named
        % 'link_vectors_copy'
        link_vectors_copy = link_vectors;
        
        % Second, divide entry number 'idx' of 'link_vectors_copy' by 2 (so
        % that it goes to the center of the link, instead of its endpoint)
        link_vectors_copy{idx} = link_vectors_copy{idx} ./ 2;
                
        % Third, use the modified 'link_vectors_copy' to get the Jacobian
        % (including the rotational component) for the midpoint of link
        % number idx
        [J_link,...
        link_ends,...
        link_end_set,...   
        link_end_set_with_base,...
        v_diff,...
        joint_axis_vectors,...
        joint_axis_vectors_R,...
        R_links,...
        J_world] = arm_Jacobian_in_link_frame(link_vectors_copy,joint_angles,joint_axes,idx);
        
        J{idx} = J_link;
    end
       

    
    %%%%%%%%%
    
    % Make a cell array 'M_links' that is the same size as the
    % 'link_vectors' cell array
    M_links = cell(size(link_vectors));
    
    % Loop over the links, transforming their inertias from the link frames
    % to the joint coordinates by pre-multiplying the 'mu'-inertia for each
    % link with the Jacobian-transpose and post-multiplying it with the
    % Jacobian, then saving the result into the corresponding entry of
    % 'M_links'
    for idx = 1:numel(M_links)
        M_links{idx} = J{idx}.' * mu_links{idx} * J{idx};
    end
                

       


    %%%%%%%%
    
    % Make an nxn matrix of zeros named 'M' (where n is the number of
    % links)
    M = zeros(numel(link_vectors));
    
    % Loop over the links, adding their inertias with respect to the joints
    % to the M matrix
    for idx = 1:numel(link_vectors)
        M = M + M_links{idx};
    end
                

       

    


end