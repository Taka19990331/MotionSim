function [mid_world, ee_world] = forward_kinematics(q1, q2, q3, l)

    rho_mid = l*cos(q1);
    z_mid   = l*sin(q1);

    rho_ee = rho_mid + l*cos(q2);
    z_ee   = z_mid   + l*sin(q2);

    mid_world = [
        rho_mid*cos(q3);
        rho_mid*sin(q3);
        z_mid
    ];

    ee_world = [
        rho_ee*cos(q3);
        rho_ee*sin(q3);
        z_ee
    ];
end