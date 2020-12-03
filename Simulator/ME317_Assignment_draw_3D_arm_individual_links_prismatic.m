function [link_vectors,...
          joint_angles,...
          joint_axes,...
          link_extensions,...
          prismatic,...
          link_colors,...
          link_set,...
          ax,...
          l,...
          l3] = ME317_Assignment_draw_3D_arm_individual_links_prismatic
% Draw the arm as a set of hexagonal prisms, with a pair of prisms
% representing an extendable link

    %%%%%%%%%%
    % Specify link vectors as a 1x3 cell array of 3x1 vectors, named
    % 'link_vectors'
    link_vectors = {[1;0;0], [1;0;0], [0;0;0.5]};
    %%%%%%%%%%
    % Specify joint angles as a 3x1 vector, named 'joint_angles'
    joint_angles = [2*pi/5; -pi/4; pi/4];
    %%%%%%%%%%
    % Specify joint axes as a 3x1 cell array, named 'joint_axes'
    joint_axes = {'z', 'y', 'x'};
    %%%%%%%%%%
    % Specify colors of links as a 1x3 cell array named 'link_colors'. Each
    % entry can be either a standard matlab color string (e.g., 'k' or 'r')
    % or a 1x3 vector of the RGB values for the color (range from 0 to 1)
    link_colors = {'b', 'k', 'b'};
    %%%%%%%%%%
    % Specify link extensions as a 3x1 vector, named 'link_extensions'
    link_extensions = [0;0.2;-0.2];
    %%%%%%%%%%
    % Specify which links are prismatic as a 3x1 vector named 'prismatic',
    % with values of 0 for a rigid link or 1 for a link that can extend
    prismatic = [0;1;1];

    %%%%%%%%%
    % Generate a cell array named 'link_set' containing start-and-end
    % points for the links, named link_set
    [link_set,...
    R_joints,...
    R_links,...
    link_set_local,...
    link_vectors_in_world,...
    links_in_world,...
    link_end_set,...
    link_end_set_with_base] = threeD_robot_arm_links_prismatic(link_vectors,joint_angles,joint_axes,link_extensions, prismatic);
    %%%%%%%%%%
    % Use arm_Jacobian to get link_ends and joint_axis_vectors_R (we will
    % use these to draw the axes onto the plot, so that it's easier to see
    % how each joint rotates). Any value of 'link_number' should be good
    % for this
    [Jac,...
    link_ends,...
    link_end_set,...   
    link_end_set_with_base,...
    v_diff,...
    joint_axis_vectors,...
    joint_axis_vectors_R] = arm_Jacobian(link_vectors,joint_angles,joint_axes,1);

  
    %%%%%%%%%
    % Create figure and axes for the plot, and store the handle in a
    % variable named 'ax'
    [ax, f] = create_axes(317);

    %%%%%%%%%
    % Draw a surface for each link, and save the handles to these surfaces
    % in a cell array named 'l'
    l = threeD_draw_links(link_set,link_colors,ax);
    %%%%%%%%%
    % Use 'view(ax,3)' to get a 3-dimensional view angle on the plot
    view(ax, 3)
    % Use axis(ax,'vis3d') to make the arm stay the same size as you rotate
    % it
    axis(ax, 'vis3d')
    %%%%%%%%%
    % Loop over the locations of the joints (the base points of
    % the links), and draw a dashed line from that joint to the point one
    % unit from the joint in the direction of that joint's axis
    %
    % Save the handles to these lines into a cell array called 'l3'.
    %
    % As you make these lines, set their color property to be the
    % same as the link immediately after the joint
    l3 = cell(size(link_set));
    for idx = 1:numel(link_set)
        r1 = link_end_set_with_base{idx};
        r2 = link_end_set{idx};
        Y = [r1, r1 + joint_axis_vectors_R{idx}];
        color = link_colors{idx};
        l3{idx} = line(ax, Y(1,:), Y(2,:), Y(3,:), 'Color', color, 'LineStyle', '--');
    end
end