function [M_total, detail] = mass_matrix(q, param)

    q1 = q(1);
    q2 = q(2);

    %% Complete rigid-link inertia

    M11_links = ...
        param.I1 ...
        + param.m1 * param.r1^2 ...
        + param.m2 * param.l1^2;

    M22_links = ...
        param.I2 ...
        + param.m2 * param.r2^2;

    M12_links = ...
        param.m2 ...
        * param.l1 ...
        * param.r2 ...
        * cos(q1 - q2);

    M_links = [
        M11_links, M12_links;
        M12_links, M22_links
    ];

    %% Reflected rotor and gear inertia

    M_motor = [
        param.JM1_ref, 0;
        0, param.JM2_ref
    ];

    %% Total inertia

    M_total = M_links + M_motor;

    %% Details

    detail.M_links = M_links;
    detail.M_motor = M_motor;

end