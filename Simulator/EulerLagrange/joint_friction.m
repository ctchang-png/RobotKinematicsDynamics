function F = joint_friction(joint_angle_velocities)
% Generate the torques acting on the joints if there is viscous friction
% present at each joint
%
% Input:
%
%   joint_angle_velocities: An nx1 vector describing the rate at which the
%       joint angles are changing
%
% Output:
%   
%   F: An nx1 vector containing the torques acting on the joints because of
%       viscous friction

    % First, define a viscous friction coefficient 'k' for the joints. Set
    % this equal to 0.001. (In a real-world scenario, you would set this
    % based on some known or measured physical properties)
    k = 0.001;
    % Generate the viscous torque vector 'F' by multipling the scaling coefficient
    % by the negative of the joint angle velocities
    F = -k * joint_angle_velocities;

end