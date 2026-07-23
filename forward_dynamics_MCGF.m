function dx = forward_dynamics_MCGF(~, x, param)
%FORWARD_DYNAMICS_MCGF Forward dynamics with M, C, G, and friction.
%
% State:
%   x = [q1; q2; dq1; dq2]
%
% Equation of motion:
%   M(q)*ddq
%   + c(q,dq)
%   + G(q)
%   + tau_fric(dq)
%   = tau
%
% For this test:
%   tau = [0; 0]

    %% State variables
    q  = x(1:2);
    dq = x(3:4);

    %% Dynamics terms
    M = mass_matrix(q, param);
    c = coriolis_torque(q, dq, param);
    G = gravity_torque(q, param);

    tau_fric = friction_torque(dq, param);

    %% Applied joint torque
    tau = [
        0;
        0
    ];

    %% Joint acceleration
    ddq = M \ ( ...
        tau ...
        - c ...
        - G ...
        - tau_fric);

    %% State derivative
    dx = [
        dq;
        ddq
    ];

end