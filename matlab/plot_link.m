function plot_link(T, axesHandle)
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
   
    % Plot the link in the specified axes
    plot3(axesHandle, frameTransformed(1, :), frameTransformed(2, :), frameTransformed(3, :), 'b', 'LineWidth', 2, 'DisplayName', 'Link');
   
 
end