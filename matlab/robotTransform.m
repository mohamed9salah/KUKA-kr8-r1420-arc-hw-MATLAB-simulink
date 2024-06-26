function T = robotTransform(DH)
    % DH is a matrix with each row representing [theta, d, a, alpha]
    
    numJoints = size(DH, 1);
    T = eye(4); % Initialize the transformation matrix
    
    for i = 1:numJoints
        alpha = DH(i, 1);
        a = DH(i, 2);
        d = DH(i, 3);
        theta = DH(i, 4);

        % Modified DH transformation matrix
        A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
             sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
             0 sin(alpha) cos(alpha) d;
             0 0 0 1];
        
        % Update the overall transformation matrix
        T = T * A;
    end
end
