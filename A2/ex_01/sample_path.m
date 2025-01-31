function [] = sample_path()

%% Plot a subset of the sample data so that the plot is less cluttered.
log = load('sample_ground_truth.mat');
theta = log.theta(2400:3200,:);
theta_dot = log.theta_dot(2400:3200,:);

%% Calculate end effector path and velocity through entire log.
%% Each vector (x, y, x_dot, and y_dot) has four values for each
%% timestep:
%% - center of mass of link 1
%% - distal end of link 1 (same as joint 2)
%% - center of mass of link 2
%% - distal end of link 2 (same as end effector)
n = length(theta);
x = zeros(n,4);
y = zeros(n,4);
x_dot = zeros(n,4);
y_dot = zeros(n,4);

% Get information about the robot:
robot = robot_info();
% Extract length of the links
l1 = robot.link_lengths(1);
l2 = robot.link_lengths(2);

% --------------- BEGIN STUDENT SECTION ----------------------------------
%% Fill in x/y positions and velocities for each point, using the forward
%% kinematics and Jacobian functions you have written.

for i = 1:n
  % Get the joint angles and joint angle velocity for this timestep, as column vectors:
  th = theta(i,:)';
  th_dot = theta_dot(i,:)';

  %% Fill in center of mass of link 1 position/velocity:
  X_distal = forward_kinematics_RR(th);
  X_distal_1 = X_distal(:,:,1);
  J_coms = jacobian_coms_RR(th);
  J_coms_1 = J_coms(:,:,1);
 
  
  x(i, 1) = 0.5.* X_distal_1(1,:) * [0;0;1];
  y(i, 1) = 0.5.* X_distal_1(2,:) * [0;0;1];
  x_dot(i, 1) = J_coms_1(1,:) * th_dot;
  y_dot(i, 1) = J_coms_1(2,:) * th_dot;
  %% Fill in distal end of link 1 position/velocity:
  J_distal = jacobian_link_ends_RR(th);
  J_distal_1 = J_distal(:,:,1);
  
  x(i, 2) = X_distal_1(1,:) * [0;0;1];
  y(i, 2) = X_distal_1(2,:) * [0;0;1];
  x_dot(i, 2) = J_distal_1(1,:) * th_dot;
  y_dot(i, 2) = J_distal_1(2,:) * th_dot;
  %% Fill in center of mass of link 2 position/velocity:
  X_distal_2 = X_distal(:,:,2);
  X_coms_2 = (X_distal_1 + X_distal_2) ./ 2;
  J_coms_2 = J_coms(:,:,2);
  
  x(i, 3) = X_coms_2(1,:) * [0;0;1];
  y(i, 3) = X_coms_2(2,:) * [0;0;1];
  x_dot(i, 3) = J_coms_2(1,:) * th_dot;
  y_dot(i, 3) = J_coms_2(2,:) * th_dot;
  %% Fill in distal end of link 2 position/velocity:
  J_distal_2 = J_distal(:,:,2);
  x(i, 4) = X_distal_2(1,:) * [0;0;1];
  y(i, 4) = X_distal_2(2,:) * [0;0;1];
  x_dot(i, 4) = J_distal_2(1,:) * th_dot;
  y_dot(i, 4) = J_distal_2(2,:) * th_dot;
end

% --------------- END STUDENT SECTION ------------------------------------

%% Plot actual data. 'quiver' is used to plot arrows along each x/y path, and
%% the arrows are centered on the x/y position.

% Sub-sample the data (resolution is how many points to skip so arrows are not
% drawn too densely)
subsample_resolution = 25;
x_sub = x(1:subsample_resolution:end,:);
y_sub = y(1:subsample_resolution:end,:);
x_dot_sub = x_dot(1:subsample_resolution:end,:);
y_dot_sub = y_dot(1:subsample_resolution:end,:);

scale = 0.75; % Scale of the arrows
figure();
hold on;
quiver(x_sub(:,1) - 0.5 * x_dot_sub(:,1), ...
       y_sub(:,1) - 0.5 * y_dot_sub(:,1), ...
       x_dot_sub(:,1), y_dot_sub(:,1), ...
       0.5, 'Color', [1, 0.5, 0.5]);
quiver(x_sub(:,2) - 0.5 * x_dot_sub(:,2), ...
       y_sub(:,2) - 0.5 * y_dot_sub(:,2), ...
       x_dot_sub(:,2), y_dot_sub(:,2), ...
       0.5, 'Color', [1, 0, 0]);
quiver(x_sub(:,3) - 0.5 * x_dot_sub(:,3), ...
       y_sub(:,3) - 0.5 * y_dot_sub(:,3), ...
       x_dot_sub(:,3), y_dot_sub(:,3), ...
       0.5, 'Color', [0.5, 0.5, 0.5]);
quiver(x_sub(:,4) - 0.5 * x_dot_sub(:,4), ...
       y_sub(:,4) - 0.5 * y_dot_sub(:,4), ...
       x_dot_sub(:,4), y_dot_sub(:,4), ...
       0.5, 'k');
hold off;
title('Plot of link positions and velocities over a sample run.');
xlabel('x [m]');
ylabel('y [m]');
axis equal;
d = 1; % Limit the range to +/- 1 meter by default
xlim([-d d]); ylim([-d d]);

end
