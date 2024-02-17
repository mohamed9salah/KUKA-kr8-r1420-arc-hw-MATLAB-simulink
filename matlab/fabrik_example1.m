function fabrik_example1()
    % DH parameters
    alpha = [0 -90 0 -90 -90 90];
    a = [1.5 0 6.1 2 0 0];
    d = [4.5 0 0 6.3 0 0.8];
    theta = [60 90 90 150 120 60];

    % Define initial and target positions
    initialPosition = [0 0 0];
    targetPosition = [2 2 2];
    
    % Calculate inverse kinematics using FABRIK
    jointAngles = fabrik_inverse_kinematics(alpha, a, d, theta, initialPosition, targetPosition);

    % Display joint angles
    disp('Joint Angles:');
    disp(jointAngles);

    % Visualize the robot arm
    visualize_robot(alpha, a, d, theta, jointAngles);
end

function jointAngles = fabrik_inverse_kinematics(alpha, a, d, theta, initialPosition, targetPosition)
    % FABRIK Inverse Kinematics Algorithm

    % Number of joints
    n = length(alpha);

    % Initial guess for joint angles
    jointAngles = zeros(1, n);

    % Set specific joint angles
    jointAngles = [113.3005 146.0851 -134.2847 148.8153 47.6493 -144.8855];

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
            targetPosHomogeneous = T_i .* targetPosHomogeneous';
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

function T = dh_matrix(alpha, a, d, theta, i, jointAngles)
    % Compute the Denavit-Hartenberg matrix for joint i
    T = [
        cosd(theta(i) + jointAngles(i)), -sind(theta(i) + jointAngles(i)) * cosd(alpha(i)), sind(theta(i) + jointAngles(i)) * sind(alpha(i)), a(i) * cosd(theta(i) + jointAngles(i));
        sind(theta(i) + jointAngles(i)), cosd(theta(i) + jointAngles(i)) * cosd(alpha(i)), -cosd(theta(i) + jointAngles(i)) * sind(alpha(i)), a(i) * sind(theta(i) + jointAngles(i));
        0, sind(alpha(i)), cosd(alpha(i)), d(i);
        0, 0, 0, 1
    ];
end

function visualize_robot(alpha, a, d, theta, jointAngles)
    % Visualize the robot arm

    % Number of joints
    n = length(alpha);

    % Initialize the plot
    figure;
    hold on;
    grid on;
    axis equal;
    view(3);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Robot Arm Visualization FABRIK method');

    % Plot robot links
    for i = 1:n
        T_i = dh_matrix(alpha, a, d, theta, i, jointAngles);
        plot_link(T_i);
        disp(['Rotation Matrix from Link ' num2str(i-1) ' to Link ' num2str(i)]);
        disp(T_i(1:3, 1:3));
    end

    % Plot end-effector point
    endEffectorPos = forward_kinematics(alpha, a, d, theta, jointAngles);
    scatter3(endEffectorPos(1), endEffectorPos(2), endEffectorPos(3), 'filled', 'r', 'DisplayName', 'End-Effector');

    % Show legend
    legend('show');
end

function endEffectorPos = forward_kinematics(alpha, a, d, theta, jointAngles)
    % Compute the forward kinematics to get the end-effector position

    % Number of joints
    n = length(alpha);

    % Initialize homogeneous transformation matrix
    T = eye(4);

    % Calculate transformation matrix iteratively
    for i = 1:n
        T_i = dh_matrix(alpha, a, d, theta, i, jointAngles);
        T = T .* T_i;
    end

    % Extract end-effector position from the matrix
    endEffectorPos = T(1:3, 4)';
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
end
