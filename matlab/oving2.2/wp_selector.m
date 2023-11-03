function [xk1, yk1, xk, yk, prev_index] = wp_selector(x, y, prev_index)

% x - north position of the ship
% y - east position of the ship

% xk1, yk1 - position of next waypoint
% xk, yk - position of last waypoint
% arrived_at_final_wp - boolean flag

min_dist = 1;

% Check if the next index is within the bounds of the waypoint array
if prev_index < length(WP)
    next_index = prev_index + 1;
    next_wp = WP(next_index, :);
    
    % Compute distance to the next waypoint
    dist = sqrt((x - next_wp(1))^2 + (y - next_wp(2))^2);

    if dist < min_dist
        prev_index = next_index;

    end

    % Extract previous and next waypoints
    xk1 = WP(next_index, 1);
    yk1 = WP(next_index, 2);

    xk = WP(prev_index, 1);
    yk = WP(prev_index, 2);
else
    % Handle the case where prev_index is out of bounds
    xk1 = x;
    yk1 = y;
    xk = x;
    yk = y;
end
