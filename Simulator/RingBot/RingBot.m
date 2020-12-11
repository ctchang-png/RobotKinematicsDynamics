classdef RingBot < handle
    % RingBot Class for storing robot information and computing
    % trajectories
    properties (Constant)
        % Density of links (i.e., mass per volume). For a prismatic joint, you should set its weight to the volume corresponding to its maximum length times link_density.
        link_density = 0.5;
        % Mass of rotational joints.
        joint_mass = 0.75; % todo
        ring_radius = 0.1;
    end
    
    properties
        link_vectors;
        prismatic;
        joint_axes;
        link_colors;
        dof;
        figure; % Handle to the plot of the arm
        initialized;
    end
    
    methods
        %% REQUIRED FUNCTIONS FOR TEMPLATE
        function obj = RingBot(link_vectors, joint_axes,prismatic, link_colors)
            %RINGROBOT Construct an instance of this class
            %
            % link_vectors: 1xn cell array of 3x1 vectors where each vector represents the link in the joint's frame
            % joint_axes: nx1 cell array with characters 'x','y',or'z' specifying the axes of each joint
            % prismatic: nx1 vector with values of 0 for a rigid link or 1 for a link that can extend
            % link_colors: nx1 cell array with colors
            
            obj.link_vectors = link_vectors;
            obj.prismatic = prismatic;
            obj.joint_axes = joint_axes;
            obj.link_colors = link_colors;
            obj.dof = numel(joint_axes);
            obj.initialized = 0;
        end
        
        function drawArm(obj, joint_angles, link_extensions)
            link_set = threeD_links_with_ring(obj.link_vectors,joint_angles,obj.joint_axes,link_extensions,obj.prismatic, obj.ring_radius);
            ax = create_axes(317);
            axis([-5 5 -5 5 0 5]); % Set axes limits
            obj.figure = threeD_draw_links(link_set,obj.link_colors,ax);
            
            view(ax,3)
            axis(ax,'vis3d');
        end
        
        function updateArm(obj, joint_angles, link_extensions)
            link_set = threeD_links_with_ring(obj.link_vectors,joint_angles,obj.joint_axes,link_extensions,obj.prismatic, obj.ring_radius);
            threeD_update_links(obj.figure, link_set);
        end
        
        function link_set = getLinksWithRing(obj, joint_angles, link_extensions)
            % Gets called in checkCollision (DO NOT EDIT)
            link_set = threeD_links_with_ring(obj.link_vectors,joint_angles,obj.joint_axes,link_extensions,obj.prismatic, obj.ring_radius);
        end
        
        function pose = fwdKin(obj, joint_angles, link_extensions, n)
           [~,...
            ~,...
            R_links,...
            ~,...
            ~,...
            ~,...
            link_end_set,...
            ~] = threeD_links_with_ring(obj.link_vectors,joint_angles,obj.joint_axes,link_extensions, obj.prismatic, obj.ring_radius);
            X = link_end_set{n};
            R = R_links{n};
            eul = rotm2eul(R, 'ZYZ');
            pose = [X; eul'];
        end
        
        function pose = fwdKin_end(obj, joint_angles, link_extensions)
            n = length(joint_angles);
            [~,...
            ~,...
            R_links,...
            ~,...
            ~,...
            ~,...
            link_end_set,...
            ~] = threeD_links_with_ring(obj.link_vectors,joint_angles,obj.joint_axes,link_extensions, obj.prismatic, obj.ring_radius);
            X = link_end_set{n};
            R = R_links{n};
            eul = rotm2eul(R, 'ZYZ');
            H = [R X; 0 0 0 1];
            X_end = H * [0.1;0;0;1];
            pose = [X_end(1:3); eul'];
        end
        
        function [J_joints, J_links] = Jacobian_end(obj,joint_angles, link_extensions)
           J_joints = zeros(6, length(joint_angles));
           J_links = zeros(6, length(link_extensions));
           n = length(joint_angles);
           pose = obj.fwdKin_end(joint_angles, link_extensions);
           eps = 1e-4;
           for ii = 1:length(joint_angles)
               joint_angles_new = joint_angles;
               joint_angles_new(ii) = joint_angles_new(ii) + eps;
               pose_new = obj.fwdKin_end(joint_angles_new, link_extensions);
               dp = (pose_new - pose);
               dp(4:6) = wrapToPi(dp(4:6));
               grad = dp/eps;
               J_joints(:,ii) = grad;
               link_extensions_new = link_extensions;
               link_extensions_new(ii) = link_extensions_new(ii) + eps;
               pose_new = obj.fwdKin_end(joint_angles, link_extensions_new);
               grad = (pose_new - pose)/eps;
               J_links(:,ii) = grad;
           end
        end
        
        
        function [joint_angles, link_extensions] = invKinNum(obj, X, R, initial_joint_angles, initial_link_extensions, max_iter)
            alpha = 0.050;
            joint_angles = initial_joint_angles;
            link_extensions = initial_link_extensions;
            eul = rotm2eul(R, 'ZYZ');
            X_star = [X ; eul'];
            thresh_x = 0.07; %max this can be is 0.10, the ring radius
            for ii = 1:max_iter
                pose = obj.fwdKin_end(joint_angles, link_extensions);
                err = (pose - X_star);
                if norm(err(1:3)) < thresh_x
                    break
                end
                [J_joints, J_links] = obj.Jacobian_end(joint_angles, link_extensions);
                h_joints = J_joints.' * err;
                h_links = J_links.' * err;
                joint_angles = joint_angles - alpha*h_joints;
                joint_angles = wrapToPi(joint_angles);
                link_extensions = link_extensions - alpha*h_links;
            end
            %{
            Rset = threeD_rotation_set(joint_angles, obj.joint_axes);
            Rlinks = rotation_set_cumulative_product(Rset);
            Reff = Rlinks{end};
            xeff = [X, X+Reff(:,1)];
            zeff = [X, X+Reff(:,3)];
            yeff = [X, X+Reff(:,2)];
            
            x = X + R(:,1);
            y = X + R(:,2);
            z = X + R(:,3);
            figure(7)
            hold on
            %{
            plot3(xeff(1,:), xeff(2,:), xeff(3,:), 'r-')
            plot3(yeff(1,:), yeff(2,:), yeff(3,:), 'g-')
            plot3(zeff(1,:), zeff(2,:), zeff(3,:), 'b-')
            %}
            plot3([X(1) x(1)], [X(2) x(2)], [X(3) x(3)], 'r--')
            plot3([X(1) y(1)], [X(2) y(2)], [X(3) y(3)], 'g--')
            plot3([X(1) z(1)], [X(2) z(2)], [X(3) z(3)], 'b--')
            %}
        end
        
        %% REQUIRED BY STUDENT TO IMPLEMENT
        function [joint_angles, link_extensions, joint_torques, link_torques] = followWire(obj, wire, initial_thetas, initial_link_extensions)
            % Generates a trajectory in theh configuration space to follow the wire
            %
            % INPUTS
            % wire:3 x n matrix where n = number of points in the wire
            % initial_thetas: dof x 1 vector
            % initial_link_extensions: dof x 1 vector
            %
            % OUTPUTS
            % joint_angles: n x m matrix of joint angles along the trajectory, 
            % where n = number joints, m = number points in trajactory.
            % link_extensions: n x m matrix of link extensions for prismatic links
            % This value is ignored for links that are not prismatic
            % joint_torques: n x m matrix of joint torques computed at the corresponding
            % configurations along the trajectory
            % link_torques: n x m matrix of prismatic link torques. (Leave 0 for
            % links that are not prismatic)
            disp("Jacobians and Inverse Kinematics are computed numerically, your computer did not crash")
            n=1;
            T = 1:n:size(wire,2);
            Tq = linspace(1,size(wire, 2), 250);
            P = wire(:,1:n:end);
            Xq = spline(T, P(1,:), Tq);
            Yq = spline(T, P(2,:), Tq);
            Zq = spline(T, P(3,:), Tq);
            traj = [Xq; Yq; Zq];
            
            joint_angles = zeros(numel(initial_thetas), length(Tq));
            joint_torques = joint_angles;
            link_extensions = zeros(numel(initial_link_extensions), length(Tq));
            link_torques = link_extensions;
            
            joint_angles(:,1) = initial_thetas;
            link_extensions(:,1) = initial_link_extensions;
            
            
            for ii = 2:(size(joint_angles, 2)-1)
                % Calculate ring pose for each waypoint
                X = traj(:,ii);     
                v = traj(:,ii+1) - traj(:,ii-1);
                z = v/norm(v);
                x = X - z*dot(X, z);
                if dot(X/norm(X), z) > cosd(1)
                    x = [1;0;0];
                    x = x - z*dot(x, z);
                    x = x/norm(x);
                    y = cross(z, x);
                else
                    x = x/norm(x);
                    y = cross(z, x);
                end
                R = [x y z];
                % Inverse Kinematics to find joint angles/extensions
                max_iters=150;
                [alpha, beta] = obj.invKinNum(X, R, joint_angles(:,ii-1), link_extensions(:,ii-1), max_iters);
                joint_angles(:,ii) = alpha;
                link_extensions(:,ii) = beta;
                
                % Joint torques based on position
                for idx = 1:numel(obj.link_vectors)
                    link_vectors_midpoint = obj.link_vectors;
                    link_extensions_midpoint = link_extensions;
                    link_vectors_midpoint{idx} = 0.5 * link_vectors_midpoint{idx};
                    link_extensions_midpoint(idx) = 0.5 * link_extensions_midpoint(idx);
                    [Jjoint_mp, Jlink_mp] = arm_Jacobian_prismatic(link_vectors_midpoint, alpha, ...
                                                                   obj.joint_axes, link_extensions_midpoint, obj.prismatic, idx);
                    [Jjoint_end, Jlink_end] = arm_Jacobian_prismatic(obj.link_vectors, alpha, ...
                                                                   obj.joint_axes, link_extensions, obj.prismatic, idx);

                    l = norm(obj.link_vectors{idx});
                    r = (l/20);
                    m_l = 9.8 * obj.link_density * l * r^2;
                    m_j = obj.joint_mass;
                    if idx == numel(obj.link_vectors)
                        m_j = 0; %mass of ring?
                    end
                    % Multiply the transpose of the {idx}th Jacobian by the force
                    % acting on the {idx} link, and save this value as 'f_torque'
                    Tjoint_mp = Jjoint_mp.' * [0;0;m_l];
                    Tjoint_end = Jjoint_end.' * [0;0;m_j];
                    Tlink_mp = Jlink_mp.' * [0;0;m_l];
                    Tlink_end = Jlink_end.' * [0;0;m_j];
                    joint_torques(:,ii) = joint_torques(:,ii) + Tjoint_mp + Tjoint_end;
                    link_torques(:,ii)  = link_torques(:,ii) + Tlink_mp + Tlink_end;
                end
            end
            joint_angles(:,1) = joint_angles(:,2);
            joint_angles(:,end) = joint_angles(:,end-1);
            link_extensions(:,1) = link_extensions(:,2);
            link_extensions(:,end) = link_extensions(:,end-1);    
                        
        end
    end
end

