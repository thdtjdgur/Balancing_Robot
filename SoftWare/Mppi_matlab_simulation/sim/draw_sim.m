function draw_sim(state, goal, obstacles, history, debug, params, step_idx, goal_distance)

clf;
hold on;
grid on;
axis equal;

xlim([-15 15]);
ylim([-15 15]);

safe_r = 0.5;

for i = 1:size(obstacles, 1)
    rectangle( ...
        'Position', [obstacles(i,1)-safe_r, obstacles(i,2)-safe_r, 2*safe_r, 2*safe_r], ...
        'Curvature', [1 1], ...
        'FaceColor', [0.2 0.2 0.2], ...
        'EdgeColor', 'none');
end

plot(goal.x, goal.y, 'rp', 'MarkerSize', 16, 'MarkerFaceColor', 'r');

if ~isempty(history)
    plot(history(:,1), history(:,2), 'b-', 'LineWidth', 2);
    plot(history(1,1), history(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
end

plot(state.x, state.y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

heading_len = 0.6;
quiver(state.x, state.y, heading_len*cos(state.psi), heading_len*sin(state.psi), ...
    0, 'Color', 'b', 'LineWidth', 1.5);

if isfield(debug, 'sampled_sequences')
    rollout_count = min(20, size(debug.sampled_sequences, 1));

    for r = 1:rollout_count
        rollout_state = state;
        xs = zeros(1, params.horizon + 1);
        ys = zeros(1, params.horizon + 1);

        xs(1) = rollout_state.x;
        ys(1) = rollout_state.y;

        for t = 1:params.horizon
            rollout_state = predict_next_state(rollout_state, debug.sampled_sequences(r, t), params);
            xs(t+1) = rollout_state.x;
            ys(t+1) = rollout_state.y;
        end

        plot(xs, ys, '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
    end
end
if isfield(debug, 'best_sequence')
    rollout_state = state;
    xs = zeros(1, numel(debug.best_sequence) + 1);
    ys = zeros(1, numel(debug.best_sequence) + 1);

    xs(1) = rollout_state.x;
    ys(1) = rollout_state.y;

    for t = 1:numel(debug.best_sequence)
        rollout_state = predict_next_state(rollout_state, debug.best_sequence(t), params);
        xs(t+1) = rollout_state.x;
        ys(t+1) = rollout_state.y;
    end

    plot(xs, ys, 'm--', 'LineWidth', 1.5);
end

title(sprintf('MPPI MATLAB Simulation | step %d | goal dist %.2f m', step_idx, goal_distance));
xlabel('x [m]');
ylabel('y [m]');

drawnow;

end