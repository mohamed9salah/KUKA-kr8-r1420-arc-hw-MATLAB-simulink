function endEffectorPos = forward_kinematics(alpha, a, d, theta, jointAngles)
    % Compute the forward kinematics to get the end-effector position

    % Number of joints
    n = length(alpha);

    % Initialize homogeneous transformation matrix
    T = eye(4);

    % Calculate transformation matrix iteratively
    for i = 1:n
        T_i = dh_matrix(alpha, a, d, theta, i, jointAngles);
        T = T * T_i;
    end

    % Extract end-effector position from the matrix
    endEffectorPos = T(1:3, 4)';
end