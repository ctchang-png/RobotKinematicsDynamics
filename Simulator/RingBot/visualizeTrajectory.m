function visualizeTrajectory(robot, wire, initial_thetas, initial_link_extensions)
    [joint_angles_traj, link_extensions_traj, joint_torques_traj, link_torques_traj] = robot.followWire(wire, initial_thetas, initial_link_extensions);
    
    % Draw theini arm in its initial position
    joint_angles = joint_angles_traj(:, 1);
    link_extensions = link_extensions_traj(:, 1);
    
    robot.drawArm(joint_angles, link_extensions);
    hold on;
    plot3(wire(1, :), wire(2, :), wire(3, :), '.');
    num_collisions = 0;
    view([90 0])
    % Animate the arm
    for idx = 1:size(joint_angles_traj,2)
        pause(0.1);
        joint_angles = joint_angles_traj(:, idx);
        link_extensions = link_extensions_traj(:, idx);
        
        
        if checkCollision(wire, robot, joint_angles, link_extensions)
            num_collisions = num_collisions + 1;
        end
        
        robot.updateArm(joint_angles, link_extensions);
    end
    
    sprintf("Collision ratio = %.2f", num_collisions/size(joint_angles_traj, 2))
    
    % Plot the joint angles
    figure(1)
    for i=1:robot.dof
        plot(joint_angles_traj(i, :));
        hold on;
    end
    hold off;
    
    % Plot the link extensions
    figure(2)
    for i=1:robot.dof
        plot(link_extensions_traj(i, :));
        hold on;
    end
    hold off;
    
    % Plot the joint torques
    figure(3)
    for i=1:robot.dof
        plot(joint_torques_traj(i, :));
        hold on;
    end
    hold off;
    
    % plot the prismatic joint torques
    figure(4)
    for i=1:robot.dof
        plot(link_torques_traj(i, :));
        hold on;
    end
    hold off;
end

