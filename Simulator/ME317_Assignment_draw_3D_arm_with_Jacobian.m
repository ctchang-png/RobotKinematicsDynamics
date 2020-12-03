function [link_vectors,...
          joint_angles,...
          joint_axes,...
          J,...
          link_ends,...
          link_end_set,...
          ax,...
          l,...
          l2,...
          l3,...
          q] = ME317_Assignment_draw_3D_arm_with_Jacobian
% Make a set of plots (subplots in one axis) that illustrate the
% relationship between the geometry of the arm and the Jacobians of the
% links

    %%%%%%%%%%
    % Specify link vectors as a 1x3 cell array of 3x1 vectors, named
    % 'link_vectors'
    link_vectors = {[1;0;0], [1;0;0], [0;0;0.5]};
    %%%%%%%%%%
    % Specify joint angles as a 3x1 vector, named 'joint_angles'
    joint_angles = [2*pi/5; -pi/4; pi/4];
    %%%%%%%%%%
    % Specify joint axes as a 3x1 cell array, named 'joint_axes'
    joint_axes = {'z','y','x'};
    %%%%%%%%%
    % Create an empty cell array named of the same size as link_vectors,
    % named J
    J = cell(size(link_vectors));
    %%%%%%%%%
    % Loop over the elements of J
    for idx = 1:numel(J)
        % In each element of J, use arm_Jacobian to find the Jacobian for
        % the corresponding link on the arm. 
        % 
        % When you call arm_Jacobian, also get link_ends, link_end_set, 
        % and joint_axis_vectors_R
        % as outputs (they will be used for plotting). These outputs are
        % independent of the link number, so you don't need to store them
        % separately for each link number -- the easy way to do this is to
        % have the link_end outputs returned in each iteration of the loop,
        % which will overwrite them every time, and leave them at their
        % value from the final iteration
        [Jac,...
        link_ends,...
        link_end_set,...   
        link_end_set_with_base,...
        v_diff,...
        joint_axis_vectors,...
        joint_axis_vectors_R] = arm_Jacobian(link_vectors,joint_angles,joint_axes,idx);
        J{idx} = Jac;
    end
        
         
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plotting
    
    %%%%%%%%%
    % Create figure and subaxes for the plot, and store the axis handles in a
    % variable named 'ax'. Use the 'create_subaxes' function you wrote
    % earlier, with the square root of the number of links for m
    % and n (round up to make these integer values), and only as many plots
    % as you have links
    p = length(link_vectors);
    m = ceil(sqrt(p));
    n = m;
    fignum = 317;
    [ax, f] = create_subaxes(fignum, m, n, p);
   

    %%%%%%%%
    % Create empty cell arrays 'l', 'l2' and 'q' to hold the handles to the
    % lines and quivers that will illustrate the arm and the components of
    % the Jacobians. Each cell array should be the same size as J.
    l = cell(size(J));
    l2 = cell(size(J));
    l3 = cell(size(J));
    q = cell(size(J));
    
    %%%%%%%%
    % Loop over the subfigure axes, drawing the robot arm into each axis.
    % Use the information in link_ends to draw a line for the arm links,
    % with circles at the endpoints, and save the handle to this line into
    % the corresponding element of 'l'
    for idx = 1:numel(ax)
        l{idx} = line(ax{idx}, link_ends(1,:), link_ends(2,:), link_ends(3,:), 'Marker', 'o');
        view(ax{idx}, 3)
        p = link_end_set{idx};
        V = J{idx};
        q{idx} = draw_vectors_at_point(p, V, ax{idx});
        l2{idx} = cell(1, size(J,2));
        l3{idx} = cell(1, size(J,2));
        for jdx = 1:idx
            r1 = link_end_set_with_base{jdx};
            r2 = link_end_set{idx};
            X = [r1, r2];
            color = q{idx}{jdx}.get('Color');
            l2{idx}{jdx} = line(ax{idx}, X(1,:), X(2,:), X(3,:), 'Color', color, 'LineStyle', ':');
            Y = [r1, r1 + joint_axis_vectors_R{jdx}];
            l3{idx}{jdx} = line(ax{idx}, Y(1,:), Y(2,:), Y(3,:), 'Color', color, 'LineStyle', '--');
        end
    end
    
    %%%%%%%%
    % Loop over the subfigure axes, making the system a 3d view in each
        
    
    %%%%%%%%
    % Loop over the subfigure axes, adding arrows for the column of the
    % corresponding Jacobian into each axis. Get the location of the end of
    % the link from link_end_set, and use the draw_vectors_at_point
    % function you wrote earlier to make the arrows. Save the handles
    % returned by draw_vectors_at_point into the corresponding elements of
    % q
    
    %%%%%%%%%
    % Loop over the subfigure axes, adding a dashed line between the end of
    % the link whose Jacobian is plotted in that axis and the joints
    % corresponding to the columns of the Jacobian
        
        % For each subfigure axis:
        %
        %   1. Make the corresponding elements of 'l2' and 'l3' empty 1xN cell
        %   arrays (where N is the number of columns in the Jacobian). This
        %   is now a nested cell array, so your command should look
        %   something like 'l2{idx} = cell(...'
        
        %   2. Loop over the locations of the joints (the base points of
        %   the links), and draw a dotted line from each of the joints that
        %   is *before* the current link to the end of the current link
        %
        %   Save the handles to these lines into the l2{idx} cell array you
        %   created for this plot (note that because this is a nested cell
        %   array, your command should look something like 
        %   'l2{idx}{idx2} = line(...'
        %
        %   As you make these lines, set their color property to be the
        %   same as the corresponding Jacobian arrow
        %   [Use get(q{idx}{idx2},'Color') to get the color of the arrow]
        
        %   3. Loop over the locations of the joints (the base points of
        %   the links), and draw a dashed line from each of the joints that
        %   is *before* the current link to the point one unit from the
        %   joint in the direction of that joint's axis
        %
        %   Save the handles to these lines into the l3{idx} cell array you
        %   created for this plot (note that because this is a nested cell
        %   array, your command should look something like 
        %   'l3{idx}{idx2} = line(...'
        %
        %   As you make these lines, set their color property to be the
        %   same as the corresponding Jacobian arrow
        %   [Use get(q{idx}{idx2},'Color') to get the color of the arrow]
                     
    
    

end