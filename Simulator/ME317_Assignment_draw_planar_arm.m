function [link_vectors,...
          joint_angles,...
          link_ends,...
          ax,...
          l] = ME317_Assignment_draw_planar_arm
% Draw the arm as one line

    %%%%%%%%%%
    % Specify link vectors as a 1x3 cell array of 2x1 vectors, named
    % 'link_vectors'
    link_vectors = cell(1,3);
    link_vectors{1} = [1;0];
    link_vectors{2} = [1;0];
    link_vectors{3} = [0.5;0];
    %%%%%%%%%%
    % Specify joint angles as a 3x1 vector, named 'joint_angles'
    joint_angles = [pi*2/5; -pi/2; pi/4];
    %%%%%%%%%%
    % Get the endpoints of the links, in a cell array named 'link_ends'
    [link_ends,...
    R_joints,...
    R_links,...
    link_vectors_in_world,...
    link_end_set,...
    link_end_set_with_base] = planar_robot_arm_endpoints(link_vectors,joint_angles);
    %%%%%%%%%
    % Create figure and axes for the plot, and store the handle in a
    % variable named 'ax'
    [ax,f] = create_axes(42069);
    %%%%%%%%
    % Draw a line from the data, with circles at the endpoints, and save
    % the handle to this line in a variable named 'l'
    l = line(ax, link_ends(1,:), link_ends(2,:), 'Marker', 'o'); 
end
