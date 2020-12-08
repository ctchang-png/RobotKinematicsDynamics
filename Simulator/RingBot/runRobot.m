link_vectors = % TODO
joint_axes = %TODO
link_colors = %TODO
prismatic = %TODO

robot = RingBot(link_vectors,joint_axes,prismatic, link_colors);
load('Wires/wire1.mat', 'wire');

% Initial values for joint angles and link extensions
joint_angles = % TODO
link_extensions = %TODO
visualizeTrajectory(robot, wire, joint_angles, link_extensions);