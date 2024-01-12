function path = discretisePath(startPoint, endPoint, n)
    % This function discretizes a path from startPoint to endPoint into n steps.
    % startPoint and endPoint are 1x3 vectors.
    % n is the number of steps.
    
    % Preallocate the path array for efficiency
    path = zeros(n, 3);

    % Calculate step size for each dimension
    step = (endPoint - startPoint) / (n - 1);

    % Generate points
    for i = 1:n
        path(i, :) = startPoint + step * (i - 1);
    end
end