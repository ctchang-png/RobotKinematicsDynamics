link_vectors = {[2.5;0;0.3], [2.5;0;0], [0;0;2.2], [0.5;0;0], [0.5;0;0]};
joint_axes = {'z', 'z', 'z', 'y', 'x'};
link_colors = {'b','k','b','k','b',[0 0.5 0]};
prismatic = [0;0;1;0;0];

robot = RingBot(link_vectors,joint_axes,prismatic, link_colors);
load('Wires/wire2.mat', 'wire');
% Initial values for joint angles and link extensions

X = wire(:,1);     
v = wire(:,2) - wire(:,1);
z = v/norm(v);
x = X - z*dot(X, z);
if dot(z, X/norm(X)) > cosd(1)
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
[initial_joint_angles, initial_link_extensions] = robot.invKinNum(X, R, [0;-pi/8;-pi/2;0;0;],...
                                                                 zeros(length(link_vectors),1), 1000);
tic()
visualizeTrajectory(robot, wire, initial_joint_angles, initial_link_extensions);
toc()