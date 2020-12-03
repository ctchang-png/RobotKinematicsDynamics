function [link_vectors,...
          joint_angles,...
          joint_axes,...
          link_ends,...
          ax,...
          l] = ME317_Assignment_draw_3D_arm
% Draw a threee-dimensional arm as one line

    %%%%%%%%%%
    % Specify link vectors as a 1x3 cell array of 3x1 vectors, named
    % 'link_vectors'
    link_vectors = {[1;0;0], [1;0;0], [0;0;0.5]};

    %%%%%%%%%%
    % Specify joint angles as a 3x1 vector, named 'joint_angles'
    joint_angles = [2*pi/5; -pi/4; pi/4];

    %%%%%%%%%%
    % Specify joint axes as a 1x3 cell array, named 'joint_axes'
    joint_axes = {'z', 'y', 'x'};

    %%%%%%%%%%
    % Get the endpoints of the links, in a cell array named 'link_ends'
    [link_ends,...
    R_joints,...
    R_links,...
    link_vectors_in_world,...
    link_end_set,...
    link_end_set_with_base] = threeD_robot_arm_endpoints(link_vectors,joint_angles,joint_axes);

    %%%%%%%%%
    % Create figure and axes for the plot, and store the handle in a
    % variable named 'ax'
    [ax, f] = create_axes(1);

    %%%%%%%%
    % Draw a line from the data, with circles at the endpoints, and save
    % the handle to this line in a variable named 'l'
    l = line(link_ends(1,:), link_ends(2,:), link_ends(3,:), 'Marker', 'o');
    
    %%%%%%%%
    % Use the view(ax,3) command to set an angled view
    view(ax, 3);
 

end