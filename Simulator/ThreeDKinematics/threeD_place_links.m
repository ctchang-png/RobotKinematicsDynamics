function link_set = threeD_place_links(links_in_world,link_end_set_with_base)
% Use the locations of the ends of a set of links to place the
% surface matrices for the links
%
% Inputs:
%
%   links_in_world: a 1xn cell array, each element of which is a 1x3 cell
%       array containing arrays the x, y, and z components for the surface
%       of the corresponding link link
%
%   link_end_set_with_base: a 1x(n+1) cell array, each element of which is
%       a vector containing the world location of the end of the
%       corresponding link
%
% Output:
%
%   link_set: a 1xn cell array, each element of which is a 1x3 cell array
%       containing arrays the x, y, and z components for the surface of
%       the corresponding link  after being translated to the location of
%       the link

    %%%%%%%%
    % Start by copying links_in_world into a new variable named 'link_set';
    link_set = links_in_world;
    %%%%%%%%
    % Loop over link_set
    for idx = 1:numel(link_set)
        % Loop over each component in the link_set
        for jdx = 1:numel(link_set{idx})
            
            % Add the corresponding component of the base of the link
            % to the surface component from link_set, 
            link_set{idx}{jdx} = link_set{idx}{jdx} + link_end_set_with_base{idx}(jdx);
        end
    end

    
end
