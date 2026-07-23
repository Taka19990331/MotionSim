function c = coriolis_torque(q, dq, param)

    q1  = q(1);
    q2  = q(2);

    dq1 = dq(1);
    dq2 = dq(2);

    %% Inertial coupling coefficient
    b = param.m2 * param.l1 * param.r2;

    %% Coriolis and centrifugal torque vector
    c = [
         b * sin(q1 - q2) * dq2^2;
        -b * sin(q1 - q2) * dq1^2
    ];

end