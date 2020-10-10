function Torques = jacobian_example()
    link_lengths = 5*ones(10,1);
    link_masses = zeros(10,1);
    joint_masses = zeros(10,1);
    ee_mass = 0;
    robot = Robot(link_lengths, link_masses, joint_masses, ee_mass);
    config = [0,0.1,0.2,0.3,0.4,0,-0.1,-0.2,-0.3,-0.4]';
    jacobians = robot.jacobians(config);
    J_ee = jacobians(:,:,end);
    W = [0, 5, 2]';
    Torques = J_ee' * W;
end