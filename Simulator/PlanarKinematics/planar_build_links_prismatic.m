function [link_set,...
    link_normals] = planar_build_links_prismatic(link_vectors,link_extensions,prismatic)
% Take a set of link vectors and augment each with a zero vector to form
% the base of the link. If the link is prismatic, represent the link as
% three lines: 
%   The base of the link is two lines that are each 3/4
% the zero-extension length of the link and are offset laterally (to the
% left and right) by a small amount from the line between the link start
% and endpoints.
%   The moving part of the link is a line 1/2 the zero-extension link
% length, and offset along the link vector by 1/2 the zero-extension length, plus
% the link extension
%
% Input:
%
%   link_vectors: a 1xn cell array, each element of which is the vector
%       from the base to end of a link, as seen in its *local* coordinate
%       frame 
%
% Outputs:
%
%   link_set: a 1xn cell array, each element of which is either:
%
%       1. A 2x2 matrix whose columns are the [0;0] base of the link in its
%       local frame and the link vector (end of the link) in its local
%       frame
%
%       or
%
%       2. A 2x8 matrix formed of three 2x2 matrices separated by 2x1 NaN
%       matrices. The columns of the three submatrices are the start and
%       endpoints of the three lines illustrating a prismatic link
%
%   link_normals: a cell array of the same size as link_vectors, containing
%       the unit normal vectors to the links


    %%%%%%%%%%%%%%
    % Use the 'cell' 'and 'size' commands to create an empty cell array the
    % same size as link_vectors named 'link_set'
    link_set = cell(size(link_vectors));
    
    %%%%%%%%%%%%%
    % Loop over the vectors in link_vectors, constructing a matrix whose
    % first column is all zeros and whose second column is the link vector,
    % and saving this matrix in the corresponding column of link_set
    for idx = 1:numel(link_set)
        v = link_vectors{idx};
        link_set{idx} = cat(2, zeros(size(v)), v);
    end
    
    %%%%%%%%%%%%%%
    % Create a normal vector for each link -- first create an empty cell
    % array the same size as link_vectors named 'link_normals', and then
    % loop over the link_vectors creating a unit-length vector that is
    % rotated 90 degrees with respect to the link vector
    
    link_normals = cell(size(link_vectors));
    for idx = 1:numel(link_normals)
        v = link_vectors{idx};
        link_normals{idx} = [0, -1; 1, 0] * v/norm(v);
    end

    %%%%%%%%%%%%
    % Peform the extra operations to make nice-looking structures for the
    % prismatic joints:
    % 
    %   Loop over all vectors in link_set
    %
    %       Check if each link is prismatic. For the prismatic links, take
    %       the link matrix and
    %
    %           1. Scale it to 3/4 the link length
    %           2. Add a vector 1/50 the link length in the link_normal
    %               direction to each column
    %           3. Make a second 3/4 scale copy of the link vector 
    %           4. Add a vector 1/50 the link length in the *negative* 
    %               link_normal direction to each column of this second
    %               matrix
    %           5. Make a 1/2 scale copy of the link vector
    %           6. Add a vector 1/2 the zero-extension link length plus the
    %               link extension amount in the direction of the link to
    %               each column of this third matrix
    %           7. Join the three resulting 2x2 matrices together into a
    %               2x8 matrix, using 2x1 NaN matrices as spacers.
    
    for idx = 1:numel(link_set)
        if ~prismatic(idx)
            continue
        end
        l = link_set{idx};
        M1 = (3/4) * l;
        M1 = M1 + (norm(l)/50) * link_normals{idx};
        
        M2 = (3/4) * l;
        M2 = M2 - (norm(l)/50) * link_normals{idx};
        
        M3 = (1/2) * l;
        v = l(:,2) - l(:,1);
        M3 = M3 + (1/2) * v + (link_extensions(idx)/norm(v)) * v;
        spacer = [NaN; NaN];
        
        link_set{idx} = cat(2, M1, spacer, M2, spacer, M3);
        
        

end