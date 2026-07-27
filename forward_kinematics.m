function [mid_world, ee_world] = forward_kinematics(q1, q2, q3, l)

    % Position in the local X-Z plane
    x_mid_local = l*cos(q1);
    z_mid_local = l*sin(q1);

    x_ee_local = x_mid_local + l*cos(q2);
    z_ee_local = z_mid_local + l*sin(q2);

    % Rotate the local X-Z plane about the world Z-axis
    mid_world = [
        x_mid_local*cos(q3);
        x_mid_local*sin(q3);
        z_mid_local
    ];

    ee_world = [
        x_ee_local*cos(q3);
        x_ee_local*sin(q3);
        z_ee_local
    ];
end