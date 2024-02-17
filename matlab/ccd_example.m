function ccd_example()
    % DH parameters
    alpha = [0 -90 0 -90 -90 90];
    a = [1.5 0 6.1 2 0 0];
    d = [4.5 0 0 6.3 0 0.8];
    theta = [60 90 90 150 120 60];

    % Define initial and target positions
    initialPosition = [0 0 0];
    targetPosition = [2 2 2];
    
    % Random initialization for joint angles
    jointAngles = rand(1, numel(alpha)) * 360 - 180;

    % Calculate inverse kinematics using CCD
    try
        jointAngles = ccd_inverse_kinematics(alpha, a, d, theta, initialPosition, targetPosition, jointAngles);
        disp('CCD method converged successfully!');
        disp('Joint Angles:');
        disp(jointAngles);
    catch ME
        disp(ME.message);
    end

    % Visualize the robot arm
    visualize_robot(alpha, a, d, theta, jointAngles);
end

function jointAngles = ccd_inverse_kinematics(alpha, a, d, theta, initialPosition, targetPosition, initialJointAngles)
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

    % Check if initialJointAngles has correct size
    if numel(initialJointAngles) ~= n
        error('Invalid initialJointAngles: The number of elements must be equal to the number of joints.');
    end

    % Initial guess for joint angles
    jointAngles = initialJointAngles;

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
function endEffectorPos = forward_kinematics(alpha, a, d, theta, jointAngles, endIndex)
    % Compute the forward kinematics to get the end-effector position

    % Number of joints
    n = length(alpha);

    if nargin < 6
        endIndex = n;
    end

    % Initialize homogeneous transformation matrix
    T = eye(4);

    % Calculate transformation matrix iteratively
    for i = 1:endIndex
        T_i = dh_matrix(alpha, a, d, theta, i, jointAngles);
        T = T .* T_i;
    end

    % Extract end-effector position from the matrix
    endEffectorPos = T(1:3, 4)';
end

function T = dh_matrix(alpha, a, d, theta, i, jointAngles)
    % Compute the Denavit-Hartenberg matrix for joint i

    % Joint angle with offset
    theta_i = theta(i) + jointAngles(i);

    % Denavit-Hartenberg parameters
    T = [
        cosd(theta_i), -sind(theta_i) * cosd(alpha(i)), sind(theta_i) * sind(alpha(i)), a(i) * cosd(theta_i);
        sind(theta_i), cosd(theta_i) * cosd(alpha(i)), -cosd(theta_i) * sind(alpha(i)), a(i) * sind(theta_i);
        0, sind(alpha(i)), cosd(alpha(i)), d(i);
        0, 0, 0, 1
    ];
end

function visualize_robot(alpha, a, d, theta, jointAngles)
    % Visualize the robot arm with rotation matrices

    % Number of joints
    n = length(alpha);

    % Create a new figure
    figure;

    % Plot robot links and display rotation matrices
    for i = 1:n
        T_i = dh_matrix(alpha, a, d, theta, i, jointAngles);
        plot_link(T_i);

        % Display the rotation matrix
        disp(['Rotation Matrix ' num2str(i-1) 'R' num2str(i) ':']);
        disp(T_i(1:3, 1:3));
    end

    % Plot end-effector point
    endEffectorPos = forward_kinematics(alpha, a, d, theta, jointAngles);
    scatter3(endEffectorPos(1), endEffectorPos(2), endEffectorPos(3), 'filled', 'r', 'DisplayName', 'End-Effector');
grid on;
title('Robot Arm Visualization CCD method');
    % Show legend
    legend('show');
end

function plot_link(T)
    % Plot a link given its homogeneous transformation matrix

    % Define the link frame
    frame = [
        0, 0, 0, 1;
        1, 0, 0, 1;
        1, 1, 0, 1;
        0, 1, 0, 1;
        0, 0, 0, 1
    ];

    % Transform the link frame
    frameTransformed = T * frame';

    % Plot the link
    plot3(frameTransformed(1, :), frameTransformed(2, :), frameTransformed(3, :), 'b', 'LineWidth', 2, 'DisplayName', 'Link');

    % Ensure visibility of the link
    hold on;

    % Draw the axes
    drawnow;
end

