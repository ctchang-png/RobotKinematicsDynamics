link_lengths = convlength([15, 13]', 'in', 'm');
robot = Robot(link_lengths, [0;0], [0;0], 0);
w = load('waypoints_2dof.mat');
waypoints = w.waypoints;
workspace_positions = zeros(2, size(waypoints, 2));
for ii = 1:size(waypoints, 2)
    X = robot.end_effector(waypoints(:,ii));
    workspace_positions(:,ii) = X(1:2);
end
segment_duration = 2; % Waypoint-to-waypoint time in seconds
use_simulator = true;

play_through_workspace_waypoints(robot, workspace_positions, segment_duration, use_simulator);