function [link_vectors,...
          joint_axes,...
          T,...
          a_start,...
          sol,...
          alpha,...
          ax,...
          link_set,...
          l,...
          link_set_history] = ME317_Assignment_falling_arm
  % Make an animated plot of a robot arm tracing a circle in the yz plane
 
    %%%%%%%%%%%%%%%%%%%%%%%
    % Specify the system structure (link vectors, link_radii, and joint
    % axes)
    n = 3;
    link_vectors = {[1;0;0], [1;0;0], [1;0;0]};
    link_radii = (1/20) * [1;1;1];
    joint_axes = {'z', 'y', 'y'};
    % Specify link vectors as a 1x3 cell array of 3x1 vectors, named
    % 'link_vectors'

    
    
    % Specify link radii as 1/20 the link length


    
    % Specify joint axes as a 3x1 cell array, named 'joint_axes'

    
    
    %%%%%%%%%%%%%%%%%%%%%%
    % Generate the system dynamics functions (inertia,
    % derivative-of-inertia, and force)
    
    % Generate the system inertia function 'M_function' as an anonymous
    % function that takes in a single input, uses that input as the
    % 'joint_angles' input to 'chain_inertia_matrix', and takes the rest of
    % the 'chain_inertia_matrix' inputs from the system information
    % specified above
    M_function = @(theta) chain_inertia_matrix(link_vectors, theta, joint_axes, link_radii);
    
    
    % Generate the system inertia derivitive function 'dM_function' by
    % applying 'inertia_derivative' to 'M_function', with the number of
    % configuration variables taken from the system information above
    dM_function = matrix_derivative(M_function, n);

    
    % Generate the forcing function on the system by constructing an
    % anonymous function that takes in time, configuration, and
    % configuration velocity, uses them to evaluate both the
    % 'gravitational_moment' and 'joint_friction' functions, and sums the
    % result. ('gravitational_moment' needs additional inputs, which should
    % be taken from the system information above)
    F_function = @(time, configuration, velocity) gravitational_moment(link_vectors,configuration,joint_axes,velocity,link_radii)...
                                                  + joint_friction(velocity);
    

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set up time-span and initial conditions for the simulation
    
    % Specify the time-span 'T' as being from zero to thirty
    T = [0 30];
    
    % Specify the starting configuration 'a_start' as being zero for all
    % joint angles
    a_start = [0;0;0];
    
    % Specify the starting velocity for the system 'adot_start' as being
    % all zeros (i.e., the system starts at rest)
    adot_start = [0;0;0];
    
    % Specify the initial state vector by stacking the a_start values on top of
    % the adot_start values
    state_start = [a_start;adot_start];
    
    %%%%%%%%%%%%%%%%%%%%%%
    % Evaluate the system motion for the specified structure, initial
    % conditions, and time-span
    
    
    % Use the 'M_function','dM_function', and 'F_function' functions
    % generated above to construct an anonymous function
    % 'state_velocity_function' that takes in time and a system state
    % vector, passes them to 'EulerLagrange_trajectory', and returns the
    % corresponding state velocity for the system
    state_velocity_function = @(time, state) EulerLagrange_trajectory(time, state, M_function, dM_function, F_function);
    
    
    % Run the 'ode45' solver with 'state_velocity_function' as the function
    % that maps from time and configuration to configuration velocity, 'T'
    % as the timespan, and 'a_start' as the initial configuration
    sol = ode45(state_velocity_function, T, state_start);
    
    
    % Use 'deval' to find the joint angles at a series of 300 times spaced
    % evenly along the interval from zero to one, saving the output to a
    % variable 'state_history'
    x = linspace(0,30,300);
    state_history = deval(sol, x);
    
    % Split 'state_history' into two components: 'alpha', with the values
    % in the top half of state_history, and 'alphadot', with the values in
    % the bottom half of state history
    alpha = state_history(1:end/2, :);
    alphadot = state_history(end/2 + 1: end, :);
    
    
    %%%%%%%%%%%%%%%%%%%%%%
    % Set up a figure in which to animate the motion of the arm as it falls
    % under gravity
    fignum = 317;
    % Create figure and axes for the plot using 'create_axes', and store
    % the handle in a variable named 'ax'
    [ax, f] = create_axes(fignum);

    % Specify colors of links as a 1x3 cell array named 'link_colors'. Each
    % entry can be either a standard matlab color string (e.g., 'k' or 'r')
    % or a 1x3 vector of the RGB values for the color (range from 0 to 1)
    link_colors = {'k', [.5 .5 .5], 'r'};

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
    
    % Draw a surface for each link, and save the handles to these surfaces
    % in a cell array named 'l'
    l = threeD_draw_links(link_set,link_colors,ax);
    
    % Use 'view(ax,3)' to get a 3-dimensional view angle on the plot
    view(ax,3)
    
    % Use axis(ax,'vis3d') to make the arm stay the same size as you rotate
    % it
    axis(ax, 'vis3d')
    
     
    % Set the axis limits to [-0.5    3   -0.85    0.85   -2    1]
    axis(ax,[-0.5	3   -0.85	0.85   -2	1])


    %%%%%%%%%%%%%%%%%%%%%%%
    % Animate the arm
    
    % For grading, create an empty 1xn cell array named 'link_set_history'
    % to hold the link surface elements at each time
    link_set_history = cell(1,size(alpha,2));
    
    
    % Loop over the columns of alpha
    for idx = 1:size(alpha, 2)
        
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