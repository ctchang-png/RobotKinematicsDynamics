function [ torque ] = get_grav_comp_torques(theta, gravity)
% get_grav_comp_torques
%
%   Calculates the joint torques required to cancel out effects due to
%   gravity.
robot_info.link_lengths = [0.381, 0.3048];
robot_info.link_masses = robot_info.link_lengths * 0.1213 + [.35, .07];
robot_info.joint_masses = [0.342, 0.342];
robot_info.end_effector_mass = 0;

% Get information about the robot:
robot = robot_info();
% Extract mass of the links, joint, and end effector [kg]
m_link_1 = robot.link_masses(1);
m_link_2 = robot.link_masses(2);
m_joint_1 = robot.joint_masses(1);
m_joint_2 = robot.joint_masses(2);
m_end_effector = robot.end_effector_mass;

% --------------- BEGIN STUDENT SECTION ----------------------------------
% Use the Jacobian to calculate the joint torques to compensate for the
% weights of the joints, links, and end effector (assuming the acceleration
% due to gravity is given be 'gravity', and it is a 2x1 (column) vector).
Jacobians_links = jacobian_coms_RR(theta);
Jacobians_ends = jacobian_link_ends_RR(theta);
J_link_1 = Jacobians_links(:,:,1);
J_link_2 = Jacobians_links(:,:,2);
J_joint_1 = zeros(2);
J_joint_2 = Jacobians_ends(:,:,1);
J_end_effector = Jacobians_ends(:,:,2);

F_link_1 = -m_link_1 * gravity;
F_link_2 = -m_link_2 * gravity;
F_joint_1 = -m_joint_1* gravity;
F_joint_2 = -m_joint_2* gravity; 
F_end_effector = -m_end_effector* gravity;

Torque = (J_link_1.')*F_link_1 + (J_link_2.')*F_link_2 +...
    (J_joint_1.')*F_joint_1 + (J_joint_2.')*F_joint_2 +...
    (J_end_effector.')*F_end_effector;
torque1 = Torque(1);
torque2 = Torque(2);

% --------------- END STUDENT SECTION ------------------------------------
% Pack into a more readable format. DO NOT CHANGE!
torque = cat(1, torque1, torque2);
end
