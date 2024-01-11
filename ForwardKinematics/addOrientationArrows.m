% Function to add orientation arrows
function addOrientationArrows(T)
    scale = 0.2; % Adjust the scale to fit your plot
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,1), scale*T(2,1), scale*T(3,1), 'r', 'LineWidth', 1.5); % X-axis
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,2), scale*T(2,2), scale*T(3,2), 'g', 'LineWidth', 1.5); % Y-axis
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,3), scale*T(2,3), scale*T(3,3), 'b', 'LineWidth', 1.5); % Z-axis
end