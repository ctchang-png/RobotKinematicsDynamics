function [link_vectors,...
          joint_axes,...
          shape_to_draw,...
          J,...
          joint_velocity,...
          T,...
          a_start,...
          sol,...
          alpha,...
          ax,...
          link_colors,...
          link_set,...
          l,...
          p,...
          l_trace,...
          link_set_history] = ME317_Assignment_trace_circle
  % Make an animated plot of a robot arm tracing a circle in the yz plane
 



    %%%%%%%%%%
    % Specify link vectors as a 1x3 cell array of 3x1 vectors, named
    % 'link_vectors'
    link_vectors = {[1;0;0], [1;0;0], [0.75;0;0]};

    %%%%%%%%%%
    % Specify joint axes as a 3x1 cell array, named 'joint_axes'
    joint_axes = {'z', 'y', 'y'};
    
    
    %%%%%%%%%%
    % Specify the function to be traced as a circle around the x axis with
    % a half-unit radius. Use the @ syntax to create handle to an anonymous
    % function named 'shape_to_draw' that takes in a time 't' and passes it
    % to the 'circle_x' function, then multiplies the result by 1/2
    shape_to_draw = @(t) circle_x(t)* (1/2);

    %%%%%%%%%%
    % Create a function 'J' that takes in a set of joint angles and returns
    % the Jacobian of the arm at those joint angles. Use the 
    % '@(b) f(a,b,c)' syntax to specify the inputs to J that should be 
    % passed to arm_Jacobian.
    J = @(alpha) arm_Jacobian(link_vectors,alpha,joint_axes,numel(link_vectors));

 
    %%%%%%%%%%
    % Pass the 'J' and 'shape_to_draw' functions to the
    % follow_trajectory function
    joint_velocity = @(t, alpha) follow_trajectory(t, alpha, J, shape_to_draw);
    
    %%%%%%%%%%%%%%
    % Set up parameters for differential equation solver
    
    % Specify the time range 'T' as being from zero to one
    T = [0 1];
    
    % Specify the starting configuration 'a_start' as being zero for the
    % first joint, pi/4 (angled down) for the second joint, and -pi/2
    % (right-angle up) for the third joint
    a_start = [0;pi/4;-pi/2];
    
    
    %%%%%%%%%
    % Run the ode solver for 'joint_velocity' as the function that maps
    % from time and configuration to configuration velocity, 'T' as the
    % timespan, and 'a_start' as the initial configuration
    sol = ode45(joint_velocity, T, a_start);
    
    
    % Use deval to find the joint angles at a series of 100 times spaced
    % evenly along the interval from zero to one, saving the output to a
    % variable 'alpha'
    alpha = deval(sol, linspace(0,1,100));

    
    
    %%%%%%%%%
    % Create figure and axes for the plot, and store the handle in a
    % variable named 'ax'
    [ax, f] = create_axes(399);

    %%%%%%%%%%
    % Specify colors of links as a 1x3 cell array named 'link_colors'. Each
    % entry can be either a standard matlab color string (e.g., 'k' or 'r')
    % or a 1x3 vector of the RGB values for the color (range from 0 to 1)
    link_colors = {'b', 'k', 'b'};

    %%%%%%%%%
    % Generate a cell array named 'link_set' containing start-and-end
    % points for the links
    [link_set,...
    R_joints,...
    R_links,...
    link_set_local,...
    link_vectors_in_world,...
    links_in_world,...
    link_end_set,...
    link_end_set_with_base] = threeD_robot_arm_links(link_vectors,a_start,joint_axes);
    
    %%%%%%%%%
    % Draw a surface for each link, and save the handles to these surfaces
    % in a cell array named 'l'
    l = threeD_draw_links(link_set, link_colors, ax);
    
    %%%%%%%%%
    % Draw the path that the end of the robot follows
    
    % Start by creating an empty matrix 'p' with three rows and as many
    % columns (time-points) as there are in alpha
    p = zeros(3, size(alpha, 2));
    
    % Next, loop over  the columns of alpha
    for idx = 1:size(alpha, 2)
        
        % For each column, calculate the endpoints for the links in the robot arm
        [link_ends,...
        R_joints,...
        R_links,...
        link_vectors_in_world,...
        link_end_set,...
        link_end_set_with_base] = threeD_robot_arm_endpoints(link_vectors,alpha(:,idx),joint_axes);
        
        % Save the endpoint of the last link into the corresponding column
        % of p
        p(:,idx) = link_ends(:,end);
    end

    
    % Finally, make a line whose data points are the rows of p, and whose
    % parent is the plotting axis, and save the handle to this line in a
    % variable 'l_trace'
    l_trace = line(ax, p(1,:), p(2,:), p(3,:), 'Color', 'k', 'LineStyle', '-');

    %%%%%%%%%
    % Use 'view(ax,3)' to get a 3-dimensional view angle on the plot
    view(ax,3)
    
    % Use axis(ax,'vis3d') to make the arm stay the same size as you rotate
    % it
    axis(ax,'vis3d')
    
     
    % Use axis(ax,'manual') to make the arm stay the same size as you move
    % the joints
    axis(ax,'manual')


    %%%%%%%%%
    % Animate the arm
    
    % For grading, create an empty 1xn cell array named 'link_set_history'
    % to hold the link surface elements at each time
    link_set_history = cell(1, size(alpha,2));

    
    % Loop over the columns of alpha
    for idx = 1:size(alpha,2)
        
        % Get the surfaces for the arm with the joint angles from
        % that column of alpha, saving them in the variable 'link_set'
        [link_set,...
        R_joints,...
        R_links,...
        link_set_local,...
        link_vectors_in_world,...
        links_in_world,...
        link_end_set,...
        link_end_set_with_base] = threeD_robot_arm_links(link_vectors,alpha(:,idx),joint_axes);      
        
        % Use the elements of link_set to update the illustration
        l = threeD_update_links(l, link_set);
        
        % Use the 'drawnow' command to make matlab update the figure before
        % going to the next step of the loop
        drawnow
        
        % Save the current link_set into the corresponding element
        % of link_set_history 
        link_set_history{idx} = link_set;
    end
        

    
    
    
 
end