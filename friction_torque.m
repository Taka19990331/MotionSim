function tau_fric = friction_torque(dq, param)
%FRICTION_TORQUE Joint friction torque.
%
% Includes:
%   1. Viscous friction
%   2. Smooth Coulomb friction
%
% The returned torque opposes joint motion.
%
% Equation of motion:
%   M(q)*ddq + c(q,dq) + G(q) + tau_fric(dq) = tau

    dq1 = dq(1);
    dq2 = dq(2);

    %% Viscous friction
    tau_viscous = [
        param.B1 * dq1;
        param.B2 * dq2
    ];

    %% Smooth Coulomb friction
    tau_coulomb = [
        param.tau_c1 * tanh(dq1 / param.v_s);
        param.tau_c2 * tanh(dq2 / param.v_s)
    ];

    %% Total friction torque
    tau_fric = ...
        tau_viscous ...
        + tau_coulomb;

end