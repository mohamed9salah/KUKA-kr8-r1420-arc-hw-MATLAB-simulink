function jointAngles = ccd_inverse_kinematics(alpha, a, d, theta, initialPosition, targetPosition)
    % CCD Inverse Kinematics Algorithm

    % Number of joints
    n = length(alpha);

    % Check if the robot arm has the correct number of joints
    if length(a) ~= n || length(d) ~= n || length(theta) ~= n || length(alpha) ~= n
        error('Invalid robot arm configuration: The number of elements in alpha, a, d, and theta must be equal to the number of joints.');
    end

    % Check if initial and target positions are 3D vectors
    if numel(initialPosition) ~= 3 || numel(targetPosition) ~= 3
        error('Invalid input: initialPosition and targetPosition must be 3D vectors.');
    end

    % Random initialization for joint angles
    jointAngles = rand(1, n) * 360 - 180;

    % Maximum iteration count and tolerance
    maxIterations = 100;
    tolerance = 1e-6;

    % Iterate through CCD iterations
    for iter = 1:maxIterations
        % Iterate through joints
        for i = 1:n
            % Calculate the end-effector position for the current joint configuration
            endEffectorPos = forward_kinematics(alpha, a, d, theta, jointAngles);

            % Calculate the vector from the current joint to the target position
            targetVector = targetPosition - endEffectorPos;

            % Calculate the vector from the current joint to the end-effector
            currentVector = forward_kinematics(alpha, a, d, theta, jointAngles, i) - endEffectorPos;

            % Calculate the rotation axis (cross product of the two vectors)
            rotationAxis = cross(currentVector, targetVector);

            % Normalize the rotation axis
            rotationAxis = rotationAxis / norm(rotationAxis);

            % Calculate the rotation angle (dot product of the two vectors)
            rotationAngle = acosd(dot(currentVector, targetVector) / (norm(currentVector) * norm(targetVector)));

            % Update the joint angle
            jointAngles(i) = jointAngles(i) + rotationAngle * sign(rotationAxis(3));

            % Check for NaN or Inf values in joint angles
            if any(isnan(jointAngles)) || any(isinf(jointAngles))
                error('Invalid joint angles. CCD method failed to converge.');
            end

            % Check convergence
            if norm(targetPosition - endEffectorPos) < tolerance
                return;
            end
        end
    end

    % If the function reaches this point, CCD did not converge
    error('CCD method did not converge within the maximum number of iterations.');
end
