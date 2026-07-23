function animate_planar_robot(t, q, param)
%ANIMATE_PLANAR_ROBOT Animate the planar two-link robot.
%
% Inputs:
%   t     : Time vector [s]
%   q     : Joint-angle history [rad]
%           q(:,1) = q1
%           q(:,2) = q2
%   param : Robot parameter structure
%
% Coordinate definition:
%   q1 and q2 are absolute link angles measured from the +X axis.
%
% Geometry:
%   Joint 1:
%       p1 = [l1*cos(q1); l1*sin(q1)]
%
%   End effector:
%       p2 = p1 + [l2*cos(q2); l2*sin(q2)]

    %% Check input

    if size(q, 2) ~= 2
        error('q must have two columns: q1 and q2.');
    end

    if length(t) ~= size(q, 1)
        error('The number of time samples must match the rows of q.');
    end

    %% Animation settings

    playback_speed = 1.0;

    % Animation frame interval [s]
    frame_interval = 1 / 30;

    % Select simulation samples nearest to fixed animation times
    animation_time = ...
        t(1):frame_interval * playback_speed:t(end);

    animation_indices = ...
        interp1( ...
            t, ...
            1:length(t), ...
            animation_time, ...
            'nearest');

    animation_indices = unique(animation_indices);

    %% Calculate all positions beforehand

    number_of_samples = length(t);

    joint_position = zeros(number_of_samples, 2);
    ee_position    = zeros(number_of_samples, 2);

    for k = 1:number_of_samples

        q1 = q(k, 1);
        q2 = q(k, 2);

        joint_position(k, :) = [
            param.l1 * cos(q1), ...
            param.l1 * sin(q1)
        ];

        ee_position(k, :) = ...
            joint_position(k, :) ...
            + [
                param.l2 * cos(q2), ...
                param.l2 * sin(q2)
            ];

    end

    %% Plot range

    total_length = param.l1 + param.l2;
    margin = 0.1 * total_length;

    axis_limit = total_length + margin;

    %% Create figure

    figure;

    hold on;
    grid on;
    axis equal;

    xlim([-axis_limit, axis_limit]);
    ylim([-axis_limit, axis_limit]);

    xlabel('X [m]');
    ylabel('Z [m]');

    title('Planar robot animation');

    %% Initial geometry

    first_index = animation_indices(1);

    base_position = [0, 0];
    joint_initial = joint_position(first_index, :);
    ee_initial    = ee_position(first_index, :);

    link_1_plot = plot( ...
        [base_position(1), joint_initial(1)], ...
        [base_position(2), joint_initial(2)], ...
        '-', ...
        'LineWidth', 4);

    link_2_plot = plot( ...
        [joint_initial(1), ee_initial(1)], ...
        [joint_initial(2), ee_initial(2)], ...
        '-', ...
        'LineWidth', 4);

    base_plot = plot( ...
        base_position(1), ...
        base_position(2), ...
        'o', ...
        'MarkerSize', 9, ...
        'MarkerFaceColor', 'auto');

    joint_plot = plot( ...
        joint_initial(1), ...
        joint_initial(2), ...
        'o', ...
        'MarkerSize', 9, ...
        'MarkerFaceColor', 'auto');

    ee_plot = plot( ...
        ee_initial(1), ...
        ee_initial(2), ...
        'o', ...
        'MarkerSize', 9, ...
        'MarkerFaceColor', 'auto');

    trajectory_plot = plot( ...
        ee_initial(1), ...
        ee_initial(2), ...
        '--', ...
        'LineWidth', 1.2);

    time_text = text( ...
        -0.95 * axis_limit, ...
         0.90 * axis_limit, ...
        sprintf('t = %.2f s', t(first_index)), ...
        'FontSize', 12);

    %% Animation loop

    previous_time = t(first_index);

    for frame = 1:length(animation_indices)

        k = animation_indices(frame);

        joint_k = joint_position(k, :);
        ee_k    = ee_position(k, :);

        %% Update links

        set(link_1_plot, ...
            'XData', [0, joint_k(1)], ...
            'YData', [0, joint_k(2)]);

        set(link_2_plot, ...
            'XData', [joint_k(1), ee_k(1)], ...
            'YData', [joint_k(2), ee_k(2)]);

        %% Update points

        set(joint_plot, ...
            'XData', joint_k(1), ...
            'YData', joint_k(2));

        set(ee_plot, ...
            'XData', ee_k(1), ...
            'YData', ee_k(2));

        %% Update trajectory

        set(trajectory_plot, ...
            'XData', ee_position(1:k, 1), ...
            'YData', ee_position(1:k, 2));

        %% Update time

        set(time_text, ...
            'String', sprintf('t = %.2f s', t(k)));

        drawnow;

        %% Keep approximately real-time playback

        if frame < length(animation_indices)

            current_time = t(k);

            pause_time = ...
                (current_time - previous_time) ...
                / playback_speed;

            pause(max(pause_time, 0));

            previous_time = current_time;

        end

    end

end