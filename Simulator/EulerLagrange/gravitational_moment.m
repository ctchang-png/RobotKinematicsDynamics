function [F,...
          link_vectors_midpoint_out,...
          J_midpoint_out,...
          m_out,...
          f_out,...
          f_torque_out] = gravitational_moment(link_vectors,joint_angles,joint_axes,joint_angle_velocities,link_radii)
% Generate the gravitational moment acting on an arm as a function of the
% arm's structure and configuration
%
% Inputs:
%
%   joint_angles: a vector of joint angles that can be passed to the
%       arm_Jacobian function
%   joint_angle_velocities: An nx1 vector describing the rate at which the
%       joint angles are changing
%   link_vectors: A 1xn cell array, each element of which is a 3x1 vector
%       describing the location of the end of a link relative to the base
%       of the link, in the link's own frame
%   joint_axes: A 1xn cell array, each element of which is a 3x1 unit
%       vector describing the axis of a joint in the frame of the link
%       preceding the joint
%   link_radii: A vector in which each element is the radius of the
%       corresponding link
%
% Output:
%
%   F: An nx1 vector containing the torques acting on the joints because of
%       the weight of the links
%
% Additonal outputs:
%
%   link_vectors_midpoint_out: A 1xn cell array, each element of which is a
%       copy of the link_vectors structure in which vector number 'idx' of
%       set number 'idx' has been divided in half, so that it gets to the
%       midpoint of the link
%   J_midpoint_out: A 1xn cell array, each element of which is the Jacobian for the
%       midpoint of the 
%   m_out: A 1xn cell array containing the masses of the links 
%   f_out{idx} = A 1xn cell array containing the gravitational forces on
%       the links;
%   f_torque_out{idx} = A 1xn cell array containing the joint torques
%       exerted by the gravitational forces on the individual links

    
    % Make a vector of zeros named 'F', the same size as the
    % 'joint_angle_velocities' vector. (This vector serves as the 'seed'
    % for building the joint torque vector, to which we will add the force
    % contributed by the weight of each link.)
    F = zeros(size(joint_angle_velocities));
    
    % Make cell arrays of the same size as 'link_vectors' named
    % 'link_vectors_midpoint_out', 'J_midpoint_out', 'm_out', 'f_out' and
    % 'f_torque_out'. (These will be used to store intermediate values in
    % your function. It would be possible -- and slightly faster not to
    % save these intermediate values, but storing them helps with debugging
    % and our grading system)
    link_vectors_midpoint_out = cell(size(link_vectors));
    J_midpoint_out = cell(size(link_vectors));
    m_out = cell(size(link_vectors));
    f_out = cell(size(link_vectors));
    f_torque_out = cell(size(link_vectors));
    
    % Loop over the links (comments below take 'idx' as the loop variable)
    for idx = 1:numel(link_vectors)
        % Make a copy of 'link_vectors' called 'link_vectors_midpoint'.
        % Then divide the {idx} entry of link_vectors_midpoint by 2 (so
        % that it goes to the midpoint of the link, rather than to the end
        % of the link).
        link_vectors_midpoint = link_vectors;
        link_vectors_midpoint{idx} = 0.5 * link_vectors_midpoint{idx};
        link_vectors_midpoint_out{idx} = link_vectors_midpoint;
        % Evaluate the Jacobian for the midpoint of the current link, and
        % call it 'J_midpoint'
        J_midpoint = arm_Jacobian(link_vectors_midpoint, joint_angles, joint_axes, idx);
        J_midpoint_out{idx} = J_midpoint;
        % Take the mass of the link as being proportional to its length
        % times the square of its radius (we're dropping a factor of pi
        % here for conveniece). Save the link mass to 'm'
        m = norm(link_vectors{idx}) * link_radii(idx)^2;
        m_out{idx} = m;
        % Take the force acting on the link as being proportional to its
        % mass, in the -z direction (here we drop the gravitational
        % acceleration g (which, together with our inertia calculation,
        % treats the product of g and the arm material density as being 1),
        % and save it to 'f'
        f = [0;0;-m];
        f_out{idx} = f;
        % Multiply the transpose of the {idx}th Jacobian by the force
        % acting on the {idx} link, and save this value as 'f_torque'
        f_torque = J_midpoint.' * f;
        f_torque_out{idx} = f_torque;
        
        % Add the f_torque for this link to the total torque F acting on
        % the joints (and save the output as the new total torque F)
        F = F + f_torque;
        % Save the current values of 'link_vectors_midpoint', 'J_midpoint', 'm',
        % 'f', and 'f_torqe' to the {idx} entries of the corresponding '_out' cell arrays
        
        
    
    
    
end