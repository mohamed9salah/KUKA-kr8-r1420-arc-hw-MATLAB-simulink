function J = calculateJacobian(alpha, a, d, theta)
    % Number of links
    n = length(alpha);

    % Initialize Jacobian matrix
    J = zeros(6, n);

    % Forward kinematics to calculate transformation matrices
    T = eye(4);
    for i = 1:n
        A = [cosd(theta(i)) -sind(theta(i)) 0 a(i);
             sind(theta(i))*cosd(alpha(i)) cosd(theta(i))*cosd(alpha(i)) -sind(alpha(i)) -sind(alpha(i))*d(i);
             sind(theta(i))*sind(alpha(i)) cosd(theta(i))*sind(alpha(i)) cosd(alpha(i)) cosd(alpha(i))*d(i);
             0 0 0 1];

        T = T * A;

        % Extract rotational and translational components
        R = T(1:3, 1:3);
        p = T(1:3, 4);

        % Populate the Jacobian matrix
        J(:, i) = [cross(R(:, 3), p); R(:, 3)];
    end
end