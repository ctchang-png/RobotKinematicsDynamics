function [link_vectors,...
          joint_angles,...
          link_extensions,...
          prismatic,...
          link_colors,...
          link_set,...
          ax,...
          l] = ME317_Assignment_draw_planar_arm_individual_links_prismatic
% Draw the arm with one line per rigid link, and three lines per prismatic
% link

    %%%%%%%%%%
    % Specify link vectors as a 1x3 cell array of 2x1 vectors, named
    % 'link_vectors'
    link_vectors = {[1;0], [1;0], [0.5;0]}; 
    %%%%%%%%%%
    % Specify joint angles as a 3x1 vector, named 'joint_angles'
    joint_angles = [2*pi/5; -pi/2; pi/4];

    %%%%%%%%%%
    % Specify link extensions as a 3x1 vector, named 'link_extensions'
    link_extensions = [0; 0.2; -0.1];
    %%%%%%%%%%
    % Specify which links are prismatic as a 3x1 vector named 'prismatic',
    % with values of 0 for a rigid link or 1 for a link that can extend
    prismatic = [0; 1; 1];

    %%%%%%%%%%
    % Specify colors of links as a 1x3 cell array named 'link_colors'. Each
    % entry can be either a standard matlab color string (e.g., 'k' or 'r')
    % or a 1x3 vector of the RGB values for the color (range from 0 to 1)
    link_colors = {'k', 'b', 'k'};

    %%%%%%%%%
    % Generate a cell array link_set containing start-and-end points for
    % the links, named 'link_set'
    [link_set,...
    R_joints,...
    R_links,...
    link_set_local,...
    link_vectors_in_world,...
    links_in_world,...
    link_end_set,...
    link_end_set_with_base] = planar_robot_arm_links_prismatic(link_vectors,joint_angles, link_extensions, prismatic);

    %%%%%%%%%
    % Create figure and axes for the plot, and store the handle in a
    % variable named 'ax'
    [ax, f] = create_axes(1);

    %%%%%%%%%
    % Use 'planar_draw_links' to draw the link set, saving the output as a
    % variable 'l'
    l = planar_draw_links(link_set,link_colors,ax);

end