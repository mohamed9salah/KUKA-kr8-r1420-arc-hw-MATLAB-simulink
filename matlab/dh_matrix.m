function T = dh_matrix(alpha, a, d, theta, i, jointAngles)
    % Compute the Denavit-Hartenberg matrix for joint i
    T = [
        cosd(theta(i) + jointAngles(i)), -sind(theta(i) + jointAngles(i)) * cosd(alpha(i)), sind(theta(i) + jointAngles(i)) * sind(alpha(i)), a(i) * cosd(theta(i) + jointAngles(i));
        sind(theta(i) + jointAngles(i)), cosd(theta(i) + jointAngles(i)) * cosd(alpha(i)), -cosd(theta(i) + jointAngles(i)) * sind(alpha(i)), a(i) * sind(theta(i) + jointAngles(i));
        0, sind(alpha(i)), cosd(alpha(i)), d(i);
        0, 0, 0, 1
    ];
end