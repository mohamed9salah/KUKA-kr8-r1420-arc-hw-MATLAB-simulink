function jointAngles = fabrik_inverse_kinematics(alpha, a, d, theta, initialPosition, targetPosition)
    % FABRIK Inverse Kinematics Algorithm

    % Number of joints
    n = length(alpha);

    % Initial guess for joint angles
    jointAngles = zeros(1, n);

    % Maximum iteration count and tolerance
    maxIterations = 100;
    tolerance = 1e-6;

    % Target position in homogeneous coordinates
    targetPosHomogeneous = [targetPosition, 1];

    % Iterate between forward and backward passes
    for iter = 1:maxIterations
        % Forward pass
        for i = 1:n
            T_i = dh_matrix(alpha, a, d, theta, i, jointAngles);
            targetPosHomogeneous = T_i .* targetPosHomogeneous;
        end

        % Backward pass
        for i = n:-1:1
            T_i = dh_matrix(alpha, a, d, theta, i, jointAngles);
            p_i = T_i(1:3, 4);
            q_i = targetPosition - p_i';
            q_i = q_i / norm(q_i);
            jointAngles(i) = atan2d(q_i(2), q_i(1));
        end

        % Check convergence
        if norm(targetPosHomogeneous(1:3) - targetPosition) < tolerance
            break;
        end
    end
end