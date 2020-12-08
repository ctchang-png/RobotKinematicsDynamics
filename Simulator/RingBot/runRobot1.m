link_vectors = {[0;0;0.2], [2.0;0;0], [1.0;0;0], [1.0;0;0], [0.5;0;0]};
joint_axes = {'z', 'y', 'x', 'z', 'x'};
link_colors = {'b','k','b','k','b',[0 0.5 0]};
prismatic = [0;1;0;0;0];

robot = RingBot(link_vectors,joint_axes,prismatic, link_colors);
load('Wires/wire3.mat', 'wire');

% Initial values for joint angles and link extensions

X = wire(:,1);     
v = wire(:,2) - wire(:,1);
z = v/norm(v);
x = X - z*dot(X, z);
x = x/norm(x);
y = cross(z, x);
R = [x y z];
[initial_joint_angles, initial_link_extensions] = robot.invKinNum(X, R, zeros(length(link_vectors),1),...
                                                                 zeros(length(link_vectors),1), 500);
tic()
visualizeTrajectory(robot, wire, initial_joint_angles, initial_link_extensions);
toc()