link_lengths = convlength([13, 10, 7]', 'in', 'm');
robot = % Fill this in
waypoints = % Fill this in. HINT: use waypoints_3dof.mat. These are joint angles at desired waypoints.
workspace_positions = % Fill this in. HINT: convert your waypoints into workspace end effector positions
segment_duration = 2; % Waypoint-to-waypoint time in seconds
use_simulator = true; 

play_through_workspace_waypoints(robot, workspace_positions, segment_duration, use_simulator);