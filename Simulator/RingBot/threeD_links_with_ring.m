function [link_set,...
            R_joints,...
            R_links,...
            link_set_local,...
            link_vectors_in_world,...
            links_in_world,...
            link_end_set,...
            link_end_set_with_base] = threeD_links_with_ring(link_vectors,joint_angles,joint_axes,link_extensions, prismatic, ring_radius)
% Take a set of link vectors and joint angles, and return a cell array of
% cell arrays, where the outer set of cells corresponds to the links, and
% the inner set contains the X, Y, and Z data for surfaces illustrating
% those links
%
% Inputs:
%
%   link_vectors: a 1xn cell array, each element of which is a 3x1 vector
%       describing the vector from the base of the corresponding link to
%       its end
%   joint_angles: a nx1 vector, each element of which is the joint angle
%       preceeding the corresponding link
%   
% Outputs:
%
%   link_set: a 1x(n+1) cell array, each element of which is 2xm matrix, whose
%       columns are the points on the surface of the associated link, and
%       the last cell contains the points for the ring.
%
% Additional outputs (These are intermediate variables. Having the option
%   to return them as outputs lets our automatic code checker tell you
%   where problems are in your code):
%
%   R_joints: The rotation matrices associated with the joints
%   R_links: The rotation matrices for the link orientations
%   link_set_local: The link vectors augmented with a zero vector that
%       represents the start of the link, including the ring.
%   link_vectors_in_world: The link vectors in their current orientations,
%   NOT including the ring.
%   links_in_world: The link start and end points in their current
%       orientations, including the ring.
%   link_end_set: The endpoints of the links after taking the cumulative
%       sum of link vectors, NOT including the ring


    %%%%%%%%
    % First, use 'threeD_rotation_set' to generate a cell array named
    % 'R_joints' that contains a set of rotation matrices corresponding to
    % the joint angles and axes
    
    R_joints = threeD_rotation_set(joint_angles,joint_axes);
    
    
    %%%%%%%%
    % Second, use 'rotation_set_cumulative_product' to generate a cell
    % array named 'R_links' that contains the orientations of the link
    % frames by taking the cumulative products of the joint rotation
    % matrices
    
    R_links = rotation_set_cumulative_product(R_joints);
    
    
    %%%%%%%%
    % Third, use 'threeD_build_links' to generate a cell array named
    % 'link_set_local' that contains the surfaces for the links (cell
    % arrays of the X, Y, and Z data for the surfaces)
    
    link_set_local = threeD_build_links_prismatic(link_vectors,link_extensions,prismatic);
    
    [X,Y,Z] = cylinder(1,20); %center axis is z
    B = orthonormal_basis_from_vector(link_vectors{end});
    X_link = X*ring_radius;
    Y_link = Y*ring_radius;
    Z_link = Z*ring_radius/20;
    X_link = X_link + ring_radius;
    ring = {X_link,Y_link,Z_link};  
    
        
    % Rotate the prism points by the orthogonal basis (to line up what
    % we were calling the x direction with the actual link vector)
    ring = rotate_surface(ring,B);         
    link_set_local{end+1} = ring;
    
    %%%%%%%%
    % Fourth, use 'vector_set_extend' to generate a cell array named
    % 'link_vectors_extended' that contains the link vectors extended by
    % the values in link_extensions
    link_vectors_extended = vector_set_extend(link_vectors,link_extensions(:).*prismatic(:));
    
    %%%%%%%%
    % Fifth, use 'vector_set_rotate' to generate a cell array named
    % 'link_vectors_in_world' that contains the (extended) link vectors
    % rotated by the rotation matrices for the links
    link_vectors_in_world = vector_set_rotate(link_vectors_extended,R_links);
    
    %%%%%%%%
    % Sixth, use 'surface_set_rotate' to generate a cell array named
    % 'links_in_world' that contains the link surface matrices rotated by
    % the rotation matrices for the links (the extension of these links
    % should already have been handled by threeD_build_links_prismatic)
    R_links{end+1} = R_links{end};
    links_in_world = surface_set_rotate(link_set_local,R_links);
    
    %%%%%%%%
    % Seventh, use 'vector_set_cumulative_sum' to generate a cell array
    % named 'link_end_set' that contains the endpoints of each link
    
    link_end_set = vector_set_cumulative_sum(link_vectors_in_world);
    
                
    %%%%%%%%
    % Eighth, add a cell containing a  zero vector (for the origin point at
    % the base of the first link) to the beginning oflink_end_set, saving
    % the result in a cell array named 'link_end_set_with_base'
    
    link_end_set_with_base = [ ...                                    Add an element to a cell array by using square brackets to concatenate two cell arays
                              {zeros(size(link_vectors{1}))}, ...     Make a zero vector the same size as the other vectors, and wrap it in a cell array
                              link_end_set ...                        Include the original cell array
                             ];

                         
    %%%%%%%%
    % Ninth, use 'threeD_place_links' to generate a cell array named
    % 'link_set' by adding the base location of each link to the surface
    % matrices of that link
    link_set = threeD_place_links(links_in_world,link_end_set_with_base);


end