function [xk1, yk1, xk, yk, prev_index] = wp_selector(x, y, prev_index, WP)

% x - north position of the ship
% y - east position of the ship

% xk1, yk1 - position of next waypoint
% xk, yk - position of last waypoint
% arrived_at_final_wp - boolean flag

min_dist = 1;
next_index = prev_index + 1;

% Only if the next index is within the bounds of the waypoint array, we
% search for next waypoint. 
if prev_index < (length(WP) - 1)
    % Compute distance to the next waypoint
    next_wp = WP(:, next_index);
    dist = sqrt((x - next_wp(1))^2 + (y - next_wp(2))^2);

    % if the ship is sufficiently close, then we move on to the next
    % waypoint. 
    if dist < min_dist
        prev_index = next_index; 
        next_index = next_index + 1;
    end
end

% Extract previous and next waypoints
xk1 = WP(1, next_index);
yk1 = WP(2, next_index);

xk = WP(1, prev_index);
yk = WP(2, prev_index);
end
