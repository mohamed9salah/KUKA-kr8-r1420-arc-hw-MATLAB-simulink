function visualize_robot_inverse(alpha, a, d, theta, jointAngles, axesHandle)
    % Visualize the robot arm with rotation matrices in the specified axes

    % Number of joints
    n = length(alpha);

    % Clear the specified axes
    cla(axesHandle);

    % Plot robot links and display rotation matrices
    for i = 1:n
        T_i = dh_matrix(alpha, a, d, theta, i, jointAngles);
        plot_link(T_i, axesHandle);

        % Display the rotation matrix
        disp(['Rotation Matrix from Link ' num2str(i-1) ' to Link ' num2str(i)]);
        disp(T_i(1:3, 1:3));
    end

    % Plot end-effector point
    endEffectorPos = forward_kinematics(alpha, a, d, theta, jointAngles);
    scatter3(axesHandle, endEffectorPos(1), endEffectorPos(2), endEffectorPos(3), 'filled', 'r', 'DisplayName', 'End-Effector');

    % Show legend
    legend(axesHandle, 'show');
end