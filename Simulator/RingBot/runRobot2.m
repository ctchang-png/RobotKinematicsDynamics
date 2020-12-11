link_vectors = {[0;0;1.25], [2.75;0;0], [1.40;0;0], [1.20;0;0], [0.1;0;0], [0.1;0;0]};
joint_axes = {'z', 'y', 'y', 'x', 'z', 'x'};
link_colors = {'b','k','b','k','b', 'k',[0 0.5 0]};
prismatic = [0;0;0;0;0;0];

robot = RingBot(link_vectors,joint_axes,prismatic, link_colors);
load('Wires/wire3.mat', 'wire');

% Initial values for joint angles and link extensions

X = wire(:,1);     
v = wire(:,2) - wire(:,1);
z = v/norm(v);
x = X - z*dot(X, z);
if dot(z, X/norm(X)) > .97
    x = [1;0;0];
    x = x - z*dot(x, z);
    x = x/norm(x);
    y = cross(z, x);
else
    x = x/norm(x);
    y = cross(z, x);
end
R = [x y z];
%Force elbow down with start condition
[initial_joint_angles, initial_link_extensions] = robot.invKinNum(X, R, [0;-pi/8;-pi/2;0;0;0],...
                                                                 zeros(length(link_vectors),1), 1000);
tic()
visualizeTrajectory(robot, wire, initial_joint_angles, initial_link_extensions);
toc()