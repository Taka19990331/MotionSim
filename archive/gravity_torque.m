function G = gravity_torque(q, param)
%GRAVITY_TORQUE Gravity torque vector.
%
% q1 and q2 are absolute link angles measured from the +X axis.
%
% Equation of motion:
%   M(q)*ddq + c(q,dq) + G(q) = tau

    q1 = q(1);
    q2 = q(2);

    G1 = ...
        (param.m1 * param.r1 ...
        + param.m2 * param.l1) ...
        * param.g ...
        * cos(q1);

    G2 = ...
        param.m2 ...
        * param.r2 ...
        * param.g ...
        * cos(q2);

    G = [
        G1;
        G2
    ];

end