function isCollision = checkCollision(wire, robot, joint_angles, link_extensions)
    % isCollision checks if a collision occurs between the robot and wire at the 
    % given configuation
    %
    % wire: 3xm matrix of wire points
    % robot: a ringBot object
    % joint_angles: a nx1 vector with joint angles
    % link_extensions: a nx1 vector with link extensions for prismatic links
    collision_thresh = 0.001;
    
    isCollision = false;
    
    % Get circle points
    link_set = robot.getLinksWithRing(joint_angles, link_extensions);
    ring_points = link_set{end};
    ring_x = [ring_points{1}(1, :)'; ring_points{1}(2, :)'];
    ring_y = [ring_points{2}(1, :)'; ring_points{2}(2, :)'];
    ring_z = [ring_points{3}(1, :)'; ring_points{3}(2, :)'];
    ring_xyz = [ring_x ring_y ring_z];
    
    % Compute centroid
    ring_center = [mean(ring_x) mean(ring_y) mean(ring_z)];
    
    % Dist between wire points
    tolerance = norm(wire(:, 1) - wire(:, 2)) * 3;
    % Check in hull
    tess = convhull(ring_x, ring_y, ring_z);
    in = inhull(wire',ring_xyz, tess, tolerance);
    for i=1:length(in)
        % Get distance from point to center of ring. 
        if in(i)
            r = norm(wire(:, i)'-ring_center);
            if abs(r - robot.ring_radius) < collision_thresh
                % If distance equals ring radius, then the wire point collides with ring 
                sprintf("Wire collides with ring at (%.2f, %.2f, %.2f) \n", wire(1, i), wire(2, i), wire(3, i))
                isCollision = true;
            end
        end
    end
    
    if sum(in) == 0
        % If ring is not on the track, also return collision
        isCollision = true;
        sprintf("Ring is not around the wire\n")
    end
    
    % Also check for each of the links
    for i=1:numel(link_set) - 1
        link_points = link_set{i};
        tolerance = 0.0001;
        link_x = [link_points{1}(1, :)'; link_points{1}(2, :)'];
        link_y = [link_points{2}(1, :)'; link_points{2}(2, :)'];
        link_z = [link_points{3}(1, :)'; link_points{3}(2, :)'];
        %remove NaNs
        link_x = rmmissing(link_x);
        link_y = rmmissing(link_y);
        link_z = rmmissing(link_z);
        
        tess = convhull(link_x, link_y, link_z);
        link_xyz = [link_x link_y link_z];
        in = inhull(wire', link_xyz, tess, tolerance);
        if sum(in) > 0
            isCollision = true;
            sprintf("Wire collides with robot arm link")
        end
    end   
end